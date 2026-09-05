// ================================================================================================
//  GpuProfiler - D3D12 timestamp queries around every GPU pass (--gpu-time).
//
//  The [rail] RENDER number is a CPU clock: in a captured run it is record+submit with the GPU's
//  execution hidden inside DumpRaw's WaitIdle, and in --bench it is record + serialized execution.
//  Neither says which PASS the GPU spent its time in. This does: two stamps around each layer's
//  Render(), a whole-frame pair, and pairs around the big internal sub-passes (compute vs draw
//  inside globe / waterbank / sea), resolved into a per-frame-in-flight readback buffer and read
//  one ring slot later -- the slot Gpu::BeginFrame has already fenced -- so it never stalls.
//
//  Off by default and free when off: the renderer carries a null pointer and issues no queries.
// ================================================================================================
#pragma once

#include "core/Gpu.h"

#include <cstdint>
#include <string>
#include <vector>

namespace ga {

class GpuProfiler {
public:
    static constexpr uint32_t kMaxPasses = 40;
    // Every Begin/End pair takes two stamps; a pass may be timed more than once per frame
    // (the durations add), so the stamp budget is generous.
    static constexpr uint32_t kMaxStamps = 128;

    void Init(Gpu& gpu);
    bool Enabled() const { return m_heap != nullptr; }

    // ---- frame lifecycle. BeginFrame first reads the slot it is about to reuse (the fence in
    // Gpu::BeginFrame guarantees that frame retired), then opens the whole-frame pair.
    // `label` is the caller's frame number for the max@frame report and the CSV row; negative
    // labels (a rail's settle frames) are recorded but excluded from the statistics.
    void BeginFrame(ID3D12GraphicsCommandList* cl, int64_t label);
    void EndFrame(ID3D12GraphicsCommandList* cl);

    // ---- a pass. Returns a token for End; a no-op token when the stamp budget is spent.
    uint32_t Begin(ID3D12GraphicsCommandList* cl, const char* name);
    void End(ID3D12GraphicsCommandList* cl, uint32_t token);

    // ---- at exit, AFTER Gpu::WaitIdle: read every slot still outstanding.
    void Drain();
    // '[gpu] <pass> mean X ms p50 Y p95 Z max W @frame N', one per pass, in first-seen order.
    // helmFrom >= 0 adds a helm-phase mean over rows with label >= helmFrom.
    void Report(int64_t helmFrom) const;
    // Per-frame rows, one column per pass (blank where the pass did not run that frame).
    bool WriteCsv(const std::string& path) const;

private:
    struct Span { uint32_t pass, begin, end; };
    struct Slot {
        bool pending = false;
        int64_t label = 0;
        uint32_t used = 0;          // stamps issued this frame
        std::vector<Span> spans;
    };
    struct Row {
        int64_t label;
        std::vector<float> ms;      // kMaxPasses wide; < 0 means the pass did not run
    };

    uint32_t PassId(const char* name);
    void ReadSlot(uint32_t slot);

    Gpu* m_gpu = nullptr;
    Com<ID3D12QueryHeap> m_heap;
    Com<ID3D12Resource> m_readback;
    double m_msPerTick = 0.0;
    Slot m_slots[Gpu::kFrameCount];
    uint32_t m_cur = 0;
    uint32_t m_frameToken = UINT32_MAX;
    bool m_open = false;
    std::vector<std::string> m_names;
    std::vector<Row> m_rows;
    // "gpu.gap": the idle between the previous frame's closing stamp and this frame's opening
    // one. Slots retire in frame order (BeginFrame reads the slot it reuses, Drain the older
    // first), so the previous row's end tick is always the frame before. In --bench it is the
    // CPU recording the next frame while the GPU is fenced idle; windowed it is the fence and
    // the present pacing; in --bench-overlap it is the bubble the shipped loop actually has.
    uint64_t m_lastFrameEnd = 0;
};

// RAII pair; null profiler = nothing recorded (the default path).
struct GpuScope {
    GpuProfiler* p;
    ID3D12GraphicsCommandList* cl;
    uint32_t tok = UINT32_MAX;
    GpuScope(GpuProfiler* prof, ID3D12GraphicsCommandList* c, const char* name) : p(prof), cl(c) {
        if (p) tok = p->Begin(cl, name);
    }
    ~GpuScope() { if (p) p->End(cl, tok); }
    GpuScope(const GpuScope&) = delete;
    GpuScope& operator=(const GpuScope&) = delete;
};

}  // namespace ga
