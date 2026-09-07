#include "core/ThreadManager.h"

#include <algorithm>

#include <objbase.h>

namespace ga {

ThreadManager& Threads() {
    static ThreadManager t;
    return t;
}

void ThreadManager::Init(int threads, int ioCap) {
    if (!m_threads.empty()) return;
    const unsigned hw = std::thread::hardware_concurrency();
    // One core stays for the frame thread, which owns the device, the queue and the command
    // list and is the thing every other thread exists to keep fed.
    const int n = threads > 0 ? threads
                              : (std::max)(4, hw > 1u ? static_cast<int>(hw) - 1 : 4);
    // M7w's number, kept exactly: "2 workers starved every descent (140 loads in 300 frames at
    // the flood-rail pose)". Moving the loader pool in here must not quietly re-tune it.
    const int io = ioCap > 0 ? ioCap
                             : (std::min)(12, (std::max)(4, hw > 2u ? static_cast<int>(hw) - 2 : 4));
    m_cap[int(Lane::Compute)] = n;
    m_cap[int(Lane::Io)] = (std::min)(io, n);
    m_quit = false;
    m_threads.reserve(static_cast<size_t>(n));
    for (int i = 0; i < n; ++i) m_threads.emplace_back([this] { Worker(); });
    Log("[jobs] pool of %d threads (Io lane capped at %d, the loader pool's own count)", n,
        m_cap[int(Lane::Io)]);
}

void ThreadManager::Shutdown() {
    if (m_threads.empty()) return;
    {
        std::lock_guard<std::mutex> lk(m_mx);
        m_quit = true;
    }
    m_cv.notify_all();
    for (auto& t : m_threads) {
        if (t.joinable()) t.join();
    }
    m_threads.clear();
    Log("[jobs] pool joined; %llu jobs submitted, hash %016llx",
        static_cast<unsigned long long>(m_submitted.load()),
        static_cast<unsigned long long>(m_hash.load()));
}

void ThreadManager::Fold(Lane lane, const char* tag) {
    // FNV-1a over (lane, tag, submission index), in submission order. Two binaries that ask for
    // the same work in the same order print the same number or the difference is real.
    //
    // Under its own lock, because a hash is a read-modify-write and two submitting threads would
    // otherwise lose each other's updates -- an instrument that is itself racy proves nothing.
    // Note what the number does and does not mean: it is only a DETERMINISTIC statement when the
    // submissions themselves are ordered, which the residency stream is (main thread, in the
    // sorted order ProcessQueues drains). Work submitted concurrently from several threads folds
    // in whatever order it arrives, and the hash says so by moving.
    const uint64_t idx = m_submitted.fetch_add(1, std::memory_order_relaxed);
    std::lock_guard<std::mutex> lk(m_hashMx);
    uint64_t h = m_hash.load(std::memory_order_relaxed);
    auto mix = [&h](uint8_t b) {
        h ^= b;
        h *= 1099511628211ull;
    };
    mix(static_cast<uint8_t>(lane));
    for (const char* p = tag ? tag : ""; *p; ++p) mix(static_cast<uint8_t>(*p));
    for (int s = 0; s < 8; ++s) mix(static_cast<uint8_t>(idx >> (s * 8)));
    m_hash.store(h, std::memory_order_relaxed);
}

void ThreadManager::Submit(Lane lane, const char* tag, std::function<void()> fn) {
    Fold(lane, tag);
    if (m_inline || m_threads.empty()) {
        fn();
        return;
    }
    {
        std::lock_guard<std::mutex> lk(m_mx);
        m_q[int(lane)].push_back({std::move(fn), lane});
    }
    m_cv.notify_one();
}

void ThreadManager::Worker() {
    // Every pool thread is a COM apartment for the life of the pool. GoogleTileProvider decodes
    // PNGs with WIC on whatever thread runs the provider, and used to do a thread_local
    // CoInitializeEx it never unwound (TileProviders.cpp). One init here, one uninit at exit,
    // and no thread is ever recycled out of an apartment mid-decode.
    const HRESULT co = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
    for (;;) {
        std::function<void()> fn;
        int lane = -1;
        {
            std::unique_lock<std::mutex> lk(m_mx);
            m_cv.wait(lk, [this] {
                if (m_quit) return true;
                for (int L = 0; L < int(Lane::kCount); ++L) {
                    if (!m_q[L].empty() && m_active[L] < m_cap[L]) return true;
                }
                return false;
            });
            if (m_quit) break;
            // Compute before Io, always: a ParallelFor's caller is BLOCKED on its chunks and a
            // paint is tens of milliseconds. Lane order is the priority.
            for (int L = 0; L < int(Lane::kCount); ++L) {
                if (m_q[L].empty() || m_active[L] >= m_cap[L]) continue;
                fn = std::move(m_q[L].front().fn);
                m_q[L].pop_front();
                ++m_active[L];
                lane = L;
                break;
            }
            if (lane < 0) continue;
        }
        fn();
        {
            std::lock_guard<std::mutex> lk(m_mx);
            --m_active[lane];
        }
        // A finished job may have unblocked a lane that was at its cap.
        m_cv.notify_all();
        m_idleCv.notify_all();
    }
    if (SUCCEEDED(co)) CoUninitialize();
}

void ThreadManager::ParallelFor(Lane lane, const char* tag, int n, int grain,
                                const std::function<void(int, int)>& fn) {
    if (n <= 0) return;
    if (grain < 1) grain = 1;
    // ONE fold for the whole call, before the inline branch. A ParallelFor is one logical
    // request for work; how many helpers it happens to raise is a scheduling detail that varies
    // with the pool's size, and folding per helper would make the hash depend on the machine's
    // core count -- useless as an A/B and different under --jobs-inline by construction.
    Fold(lane, tag);
    if (m_inline || m_threads.empty()) {
        for (int j = 0; j < n; j += grain) fn(j, (std::min)(n, j + grain));
        return;
    }
    // THE WAIT IS ON WORK FINISHED, NOT ON HELPERS RUNNING, and the state is OWNED by the jobs.
    // Both of those are scars. Waiting on a helper's completion deadlocks the moment a helper is
    // queued behind saturated workers and never gets scheduled: the caller drains every chunk
    // itself, the work is done, and it then waits forever for a job that will not run. And the
    // helpers must not capture this stack frame -- once the wait is on work rather than helpers,
    // a queued helper can outlive the call and would dereference a dead `fn`. Both were caught
    // by ThreadTest.cpp case 7(b), which parks every worker and then calls in here.
    struct Shared {
        std::atomic<int> next{0};
        std::atomic<int> done{0};
        int chunks = 0, n = 0, grain = 0;
        std::function<void(int, int)> fn;
        std::mutex mx;
        std::condition_variable cv;
    };
    auto st = std::make_shared<Shared>();
    st->chunks = (n + grain - 1) / grain;
    st->n = n;
    st->grain = grain;
    st->fn = fn;

    auto drain = [st] {
        for (;;) {
            const int c = st->next.fetch_add(1, std::memory_order_relaxed);
            if (c >= st->chunks) return;
            const int j0 = c * st->grain;
            st->fn(j0, (std::min)(st->n, j0 + st->grain));
            if (st->done.fetch_add(1, std::memory_order_acq_rel) + 1 == st->chunks) {
                std::lock_guard<std::mutex> l(st->mx);   // pairs with the predicate wait below
                st->cv.notify_all();
            }
        }
    };

    const int helpers = (std::min)(st->chunks - 1, m_cap[int(lane)]);
    if (helpers > 0) {
        {
            std::lock_guard<std::mutex> lk(m_mx);
            for (int i = 0; i < helpers; ++i) m_q[int(lane)].push_back({drain, lane});
        }
        m_cv.notify_all();
    }

    drain();   // the caller does its share, so progress never depends on a free worker

    // Every chunk is claimed by the time drain() returns, but one may still be running on a
    // helper, so the completion count is what settles it.
    std::unique_lock<std::mutex> lk(st->mx);
    st->cv.wait(lk, [&st] { return st->done.load(std::memory_order_acquire) >= st->chunks; });
}

}  // namespace ga
