#include "core/GpuProfiler.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

namespace ga {

void GpuProfiler::Init(Gpu& gpu) {
    m_gpu = &gpu;
    D3D12_QUERY_HEAP_DESC qd{};
    qd.Type = D3D12_QUERY_HEAP_TYPE_TIMESTAMP;
    qd.Count = Gpu::kFrameCount * kMaxStamps;
    GA_CHECK(gpu.Device()->CreateQueryHeap(&qd, IID_PPV_ARGS(&m_heap)));
    m_heap->SetName(L"gpu-time query heap");

    D3D12_HEAP_PROPERTIES hp{};
    hp.Type = D3D12_HEAP_TYPE_READBACK;
    D3D12_RESOURCE_DESC rd{};
    rd.Dimension = D3D12_RESOURCE_DIMENSION_BUFFER;
    rd.Width = uint64_t(Gpu::kFrameCount) * kMaxStamps * sizeof(uint64_t);
    rd.Height = 1;
    rd.DepthOrArraySize = 1;
    rd.MipLevels = 1;
    rd.SampleDesc.Count = 1;
    rd.Layout = D3D12_TEXTURE_LAYOUT_ROW_MAJOR;
    GA_CHECK(gpu.Device()->CreateCommittedResource(&hp, D3D12_HEAP_FLAG_NONE, &rd,
                                                   D3D12_RESOURCE_STATE_COPY_DEST, nullptr,
                                                   IID_PPV_ARGS(&m_readback)));
    m_readback->SetName(L"gpu-time readback");

    UINT64 freq = 0;
    GA_CHECK(gpu.Queue()->GetTimestampFrequency(&freq));
    m_msPerTick = freq ? 1000.0 / double(freq) : 0.0;
    for (auto& s : m_slots) s.spans.reserve(kMaxStamps / 2);
    Log("[gpu] --gpu-time: timestamp queries armed (%u stamps x %u frames in flight, %.3f ns/tick)",
        kMaxStamps, Gpu::kFrameCount, freq ? 1e9 / double(freq) : 0.0);
}

uint32_t GpuProfiler::PassId(const char* name) {
    for (uint32_t i = 0; i < m_names.size(); ++i) {
        if (m_names[i] == name) return i;
    }
    if (m_names.size() >= kMaxPasses) return UINT32_MAX;
    m_names.emplace_back(name);
    return uint32_t(m_names.size() - 1);
}

void GpuProfiler::BeginFrame(ID3D12GraphicsCommandList* cl, int64_t label) {
    m_cur = m_gpu->FrameIndex();
    // Gpu::BeginFrame just waited on the fence of the frame that last used this index, so its
    // resolve has landed: read it now, then reuse the slot.
    if (m_slots[m_cur].pending) ReadSlot(m_cur);
    Slot& s = m_slots[m_cur];
    s.pending = false;
    s.label = label;
    s.used = 0;
    s.spans.clear();
    m_open = true;
    // "whole-frame", not "frame": the CSV's first column is the frame index and a pass of the
    // same name would shadow it for anything that reads the header by name.
    m_frameToken = Begin(cl, "whole-frame");
}

uint32_t GpuProfiler::Begin(ID3D12GraphicsCommandList* cl, const char* name) {
    if (!m_open) return UINT32_MAX;
    Slot& s = m_slots[m_cur];
    if (s.used + 2 > kMaxStamps) return UINT32_MAX;
    const uint32_t pass = PassId(name);
    if (pass == UINT32_MAX) return UINT32_MAX;
    const uint32_t local = s.used;
    s.used += 2;
    s.spans.push_back({pass, local, local + 1});
    cl->EndQuery(m_heap.Get(), D3D12_QUERY_TYPE_TIMESTAMP, m_cur * kMaxStamps + local);
    return uint32_t(s.spans.size() - 1);
}

void GpuProfiler::End(ID3D12GraphicsCommandList* cl, uint32_t token) {
    if (!m_open || token == UINT32_MAX) return;
    Slot& s = m_slots[m_cur];
    if (token >= s.spans.size()) return;
    cl->EndQuery(m_heap.Get(), D3D12_QUERY_TYPE_TIMESTAMP, m_cur * kMaxStamps + s.spans[token].end);
}

void GpuProfiler::EndFrame(ID3D12GraphicsCommandList* cl) {
    if (!m_open) return;
    End(cl, m_frameToken);
    Slot& s = m_slots[m_cur];
    if (s.used) {
        cl->ResolveQueryData(m_heap.Get(), D3D12_QUERY_TYPE_TIMESTAMP, m_cur * kMaxStamps, s.used,
                             m_readback.Get(), uint64_t(m_cur) * kMaxStamps * sizeof(uint64_t));
        s.pending = true;
    }
    m_open = false;
}

void GpuProfiler::ReadSlot(uint32_t slot) {
    Slot& s = m_slots[slot];
    s.pending = false;
    if (!s.used) return;
    const uint64_t base = uint64_t(slot) * kMaxStamps * sizeof(uint64_t);
    D3D12_RANGE range{SIZE_T(base), SIZE_T(base + s.used * sizeof(uint64_t))};
    void* p = nullptr;
    if (FAILED(m_readback->Map(0, &range, &p)) || !p) return;
    const uint64_t* ticks = reinterpret_cast<const uint64_t*>(static_cast<const uint8_t*>(p) + base);
    Row row;
    row.label = s.label;
    row.ms.assign(kMaxPasses, -1.0f);
    for (const Span& sp : s.spans) {
        const uint64_t t0 = ticks[sp.begin], t1 = ticks[sp.end];
        const float ms = float(double(t1 >= t0 ? t1 - t0 : 0) * m_msPerTick);
        if (row.ms[sp.pass] < 0.0f) row.ms[sp.pass] = ms;
        else row.ms[sp.pass] += ms;
    }
    // The whole-frame pair is spans[0] (BeginFrame opens it first); its raw ticks give the
    // inter-frame gap against the previous row's closing stamp (see m_lastFrameEnd).
    if (!s.spans.empty()) {
        const uint64_t fb = ticks[s.spans[0].begin], fe = ticks[s.spans[0].end];
        if (m_lastFrameEnd && fb > m_lastFrameEnd) {
            const uint32_t gap = PassId("gpu.gap");
            if (gap != UINT32_MAX) {
                row.ms[gap] = float(double(fb - m_lastFrameEnd) * m_msPerTick);
            }
        }
        m_lastFrameEnd = fe;
    }
    D3D12_RANGE none{0, 0};
    m_readback->Unmap(0, &none);
    m_rows.push_back(std::move(row));
}

void GpuProfiler::Drain() {
    // Slots retire in submission order; read the OLDER one first so the rows stay in order.
    for (uint32_t k = 1; k <= Gpu::kFrameCount; ++k) {
        const uint32_t slot = (m_cur + k) % Gpu::kFrameCount;
        if (m_slots[slot].pending) ReadSlot(slot);
    }
}

void GpuProfiler::Report(int64_t helmFrom) const {
    if (m_names.empty()) { Log("[gpu] --gpu-time: no passes recorded"); return; }
    size_t counted = 0;
    for (const Row& r : m_rows) if (r.label >= 0) ++counted;
    Log("[gpu] GPU time per pass over %zu frames (timestamp queries; sub-passes are inside their "
        "layer's number)%s", counted,
        helmFrom >= 0 ? " -- last column: mean over the helm phase" : "");
    Log("[gpu] %-20s %9s %9s %9s %9s %8s%s", "pass", "mean", "p50", "p95", "max", "@frame",
        helmFrom >= 0 ? "     helm" : "");
    for (uint32_t p = 0; p < m_names.size(); ++p) {
        std::vector<float> v;
        v.reserve(m_rows.size());
        double sum = 0.0, helmSum = 0.0;
        size_t helmN = 0;
        float mx = -1.0f;
        int64_t mxAt = -1;
        for (const Row& r : m_rows) {
            if (r.label < 0 || r.ms[p] < 0.0f) continue;
            v.push_back(r.ms[p]);
            sum += r.ms[p];
            if (r.ms[p] > mx) { mx = r.ms[p]; mxAt = r.label; }
            if (helmFrom >= 0 && r.label >= helmFrom) { helmSum += r.ms[p]; ++helmN; }
        }
        if (v.empty()) continue;
        std::sort(v.begin(), v.end());
        auto pct = [&](double q) {
            return v[(std::min)(v.size() - 1, size_t(q * double(v.size() - 1) + 0.5))];
        };
        if (helmFrom >= 0) {
            Log("[gpu] %-20s %6.3f ms %6.3f ms %6.3f ms %6.3f ms %8lld %6.3f ms  (%zu frames)",
                m_names[p].c_str(), sum / double(v.size()), pct(0.50), pct(0.95), mx,
                static_cast<long long>(mxAt), helmN ? helmSum / double(helmN) : 0.0, v.size());
        } else {
            Log("[gpu] %-20s %6.3f ms %6.3f ms %6.3f ms %6.3f ms %8lld  (%zu frames)",
                m_names[p].c_str(), sum / double(v.size()), pct(0.50), pct(0.95), mx,
                static_cast<long long>(mxAt), v.size());
        }
    }
    for (const std::string& n : m_names) {
        if (n != "gpu.gap") continue;
        Log("[gpu] gpu.gap is the idle between one frame's closing stamp and the next frame's "
            "opening one (outside whole-frame): the GPU waiting on the CPU record, the "
            "BeginFrame fence, or the display's present pacing");
    }
}

bool GpuProfiler::WriteCsv(const std::string& path) const {
    FILE* f = nullptr;
    if (fopen_s(&f, path.c_str(), "w") != 0 || !f) return false;
    fprintf(f, "frame");
    for (const std::string& n : m_names) fprintf(f, ",%s", n.c_str());
    fprintf(f, "\n");
    for (const Row& r : m_rows) {
        fprintf(f, "%lld", static_cast<long long>(r.label));
        for (uint32_t p = 0; p < m_names.size(); ++p) {
            if (r.ms[p] < 0.0f) fprintf(f, ",");
            else fprintf(f, ",%.4f", double(r.ms[p]));
        }
        fprintf(f, "\n");
    }
    fclose(f);
    Log("[gpu] per-frame GPU series -> %s", path.c_str());
    return true;
}

}  // namespace ga
