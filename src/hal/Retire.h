// ================================================================================================
//  Retire.h - M12 step 3e: TEMPORAL OWNERSHIP, as one queue. Anything a recorded command still
//  reads -- an upload allocation, a replaced pipeline, a tile being NULL-mapped, a landing
//  slot -- retires on the fence, never on scope exit (Context.h's ownership note). This is
//  that rule as a mechanism: Retire(release, fenceValue) queues a release to run once the frame
//  fence has passed `fenceValue`; DrainRetired(completed) runs the due ones in the order they
//  were queued, and Gpu::BeginFrame calls it once per frame with the fence's completed value,
//  right after it has waited for the frame it is about to reuse -- so a release keyed to that
//  frame's fence runs before anything is recorded over it. Thread-safe to queue from anywhere;
//  the releases run on the frame thread. A release still queued at exit never runs.
//
//  WHAT IS NOT ROUTED THROUGH IT, AND WHY. The residency manager keeps three deferred releases
//  of its own, every one keyed by TURN COUNT, not by fence: a dropped tile's NULL map
//  (m_retiring, kEvictAgeFrames = 4 turns after the drop; ProcessQueues' retire loop), the
//  DirectStorage landing slots (m_stageRetire, kStageRetireFrames = 4) and the evictor's age
//  rule (kEvictAgeFrames since last seen). Four turns is longer than the two-frame ring's
//  overlap on purpose -- it also covers the manager's own recorded-but-not-executed copies --
//  and the exact settle counts on that number (Residency.h settleExact: a still is dumped
//  kEvictAgeFrames + 4 exact turns after the last drop). Re-keying any of them to the fence
//  would unmap a tile on a different frame, which is a different resident set at a different
//  time; so none is routed here, and this step states the contract and hands over the queue.
//  The upload path needs none (EndUpload waits). The shader reload swaps its pipelines under
//  a WaitIdle (Renderer::ReloadShaders), which is the whole reason an immediately released
//  pipeline has never been read by a list in flight: Pipeline.h's banner credits the frame ring
//  with holding that reference, but the ring holds none (Gpu.h: an allocator and a list per
//  frame; D3D12 tracks no lifetimes for objects a command list recorded), so the WaitIdle is
//  the guarantee. The first release routed here will be the one that removes such a wait; it
//  needs the fence value the recording in progress is going to signal, which Gpu does not
//  expose yet (EndFrame signals ++m_fenceValue) -- one accessor, added with that release.
//
//  Header-only, one queue for the process: the frame ring is one, and so is its fence.
// ================================================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <functional>
#include <mutex>
#include <utility>
#include <vector>

namespace ga::hal {

namespace retire_detail {
struct Queue {
    std::mutex mx;
    std::vector<std::pair<uint64_t, std::function<void()>>> due;
};
inline Queue& Q() {
    static Queue q;
    return q;
}
}  // namespace retire_detail

// Run `release` once the frame fence has passed `fenceValue`.
inline void Retire(std::function<void()> release, uint64_t fenceValue) {
    retire_detail::Queue& q = retire_detail::Q();
    std::lock_guard<std::mutex> lk(q.mx);
    q.due.emplace_back(fenceValue, std::move(release));
}

// Run every queued release whose fence value is <= completedFence, in queue order. Called by
// Gpu::BeginFrame; a release may Retire() again (it runs outside the lock).
inline void DrainRetired(uint64_t completedFence) {
    std::vector<std::function<void()>> run;
    {
        retire_detail::Queue& q = retire_detail::Q();
        std::lock_guard<std::mutex> lk(q.mx);
        std::vector<std::pair<uint64_t, std::function<void()>>> keep;
        for (auto& [fence, release] : q.due) {
            if (fence <= completedFence) run.push_back(std::move(release));
            else keep.emplace_back(fence, std::move(release));
        }
        q.due.swap(keep);
    }
    for (auto& r : run) r();
}

// The instrument: releases still waiting on their fence.
inline size_t RetiredPending() {
    retire_detail::Queue& q = retire_detail::Q();
    std::lock_guard<std::mutex> lk(q.mx);
    return q.due.size();
}

}  // namespace ga::hal
