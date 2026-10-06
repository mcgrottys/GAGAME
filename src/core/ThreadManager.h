// ThreadManager -- ONE pool, for every thread in the process that is not the frame thread.
//
// What it replaces, counted on this machine (16 logical cores) before it existed:
//
//     ResidencyManager loaders     min(12, max(4, hw-2)) = 12   persistent, own mutex + cv
//     WaveField::ParallelRows      max(4, hw-2) = 14            SPAWNED AND JOINED PER CALL,
//                                                               3 call sites, nested inside...
//     WaveField::SolveAsync        1                            ...this one
//     GlobeLayer predict worker    1                            own mutex + cv
//     WaterSceneWatch              1                            blocks in ReadDirectoryChangesW
//
// About 29 threads on 16 cores during a bucket roll, with no shared admission control, no
// priority between them, and five separate shutdown disciplines -- two of which (residency,
// WaveField) have no destructor at all and are joined by hand at six exit sites between them.
// There was no job, task or parallel-for abstraction anywhere in src/: ParallelRows was the
// only fork-join in the codebase and it allocated threads to do it.
//
// THIS BUYS NO FRAMERATE AND DOES NOT PRETEND TO. Measured 2026-09-06 on the storm rail: the
// GPU is 6.187 ms of which globe.mesh is 4.908 (79%), so perfect threading takes the shipped
// loop from ~130 to ~160 fps and then stops, GPU-bound. This is for correctness and for having
// one place where thread policy lives.
//
// LANES exist because the work is not interchangeable. A residency load is a 64 KB read or a
// tens-of-milliseconds paint; a ParallelFor chunk is microseconds and its caller is BLOCKED
// waiting for it. Sharing one FIFO would put the short work behind the long work.
//
//   Lane::Io       archive reads and tile paints. Capped at the loader pool's old worker
//                  count and served strictly FIFO, so the residency manager's request stream
//                  reaches the disk in exactly the order and concurrency it used to.
//   Lane::Compute  short fork/join work. Any pool thread; served before Io so a blocked
//                  ParallelFor is never stuck behind a paint.
//   Lane::Long     F13: work that runs for seconds -- the wave solve and its row helpers, the
//                  prefill, the current's resample. Served last, and with Io it leaves
//                  kReserve threads free, so the frame's short work always finds one: measured,
//                  the predicted walk's join waited 60-190 ms behind the solve's rows when the
//                  solve was Compute work.
//
// DETERMINISM. Every submission is folded into an FNV-1a over (lane, tag, index) in submission
// order -- JobsHash(), printed as [jobs] -- and --jobs-inline runs every Submit and ParallelFor
// on the calling thread in submission order. That is the same shape as --predict-inline and the
// predicted-stream hash that gated the prefetch walk moving off the frame thread (cfc8e50).
// What it does NOT make deterministic, and no hash here should be read as claiming: the ORDER
// LOADS LAND. Priors 30 already establishes that the resident set at the pool cap is decided by
// landing order. This hashes what was ASKED, not what arrived first.
#pragma once

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "Common.h"

namespace ga {

enum class Lane : int { Compute = 0, Io = 1, Long = 2, kCount = 3 };

class ThreadManager {
public:
    ~ThreadManager() { Shutdown(); }

    // `ioCap` is the concurrency the Io lane is allowed -- the residency loader pool's old
    // worker count, so moving it here changes neither its order nor its parallelism.
    void Init(int threads = 0, int ioCap = 0);
    void Shutdown();

    bool Running() const { return !m_threads.empty(); }
    size_t ThreadCount() const { return m_threads.size(); }
    int IoCap() const { return m_cap[int(Lane::Io)]; }

    // Run everything on the calling thread, in submission order. The A/B for every lane.
    void SetInline(bool on) { m_inline = on; }
    bool Inline() const { return m_inline; }

    uint64_t JobsHash() const { return m_hash.load(std::memory_order_relaxed); }
    uint64_t JobsSubmitted() const { return m_submitted.load(std::memory_order_relaxed); }

    // Fire and forget. The job must own everything it touches, or outlive nothing.
    void Submit(Lane lane, const char* tag, std::function<void()> fn);

    // Blocking fork/join over [0, n). THE CALLER PARTICIPATES, which is what makes it safe to
    // call from inside a pool job: the work always advances on this thread even when every
    // other thread is busy, so a nested ParallelFor cannot deadlock waiting for a free worker.
    // This is what lets WaveField::ParallelRows stop spawning 14 threads per call.
    void ParallelFor(Lane lane, const char* tag, int n, int grain,
                     const std::function<void(int, int)>& fn);

private:
    struct Job {
        std::function<void()> fn;
        Lane lane;
    };
    void Worker();
    void Fold(Lane lane, const char* tag);

    std::vector<std::thread> m_threads;
    std::deque<Job> m_q[int(Lane::kCount)];
    int m_active[int(Lane::kCount)] = {0, 0, 0};
    int m_cap[int(Lane::kCount)] = {0, 0, 0};
    static constexpr int kReserve = 2;   // F13: threads Io and Long together leave to Compute
    // A lane's job may start: under its own cap, and (Io, Long) under the pool less the reserve.
    bool MayStart(int L) const {
        if (m_active[L] >= m_cap[L]) return false;
        if (L == int(Lane::Compute)) return true;
        return m_active[int(Lane::Io)] + m_active[int(Lane::Long)] <
               static_cast<int>(m_threads.size()) - kReserve;
    }
    std::mutex m_mx;
    std::condition_variable m_cv;      // work available, or quitting
    bool m_quit = false;
    bool m_inline = false;
    mutable std::mutex m_hashMx;   // Fold() is a read-modify-write; see ThreadManager.cpp
    std::atomic<uint64_t> m_hash{1469598103934665603ull};   // FNV-1a offset basis
    std::atomic<uint64_t> m_submitted{0};
};

// The process's pool. Init() from main before anything submits; Shutdown() from main at exit.
ThreadManager& Threads();

}  // namespace ga
