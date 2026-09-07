// ThreadAudit -- the instrument that says whether the sparse GA tree is actually thread safe,
// instead of assuming it from the mutexes that exist.
//
// docs/SPARSE_GA.md 43 states the contract: "Parents are read-modify-written by painting threads
// under a per-tree recursive mutex." That mutex (TileTree::m_foldMx) covers FoldUp on ONE node.
// It does not cover LeafTile's own write, it does not cover any reader, and DropCachedAddress
// deletes files on the PARENT node under no lock at all. A tile file is published by
// std::ofstream -- open truncates to zero, the bytes flush at close -- so a reader between those
// two instants sees a short file, ReadTile returns false, and the caller treats false as ABSENT
// and repaints the parent from the source, discarding every sibling's fold already folded into
// it. That is silent pyramid data loss, decided by timing.
//
// This header does not fix any of that. It COUNTS it, on the shipped code, so the fix has a
// number to be gated against. Off by default; --thread-audit turns it on and the run prints
// [thread-audit] at exit. Everything is one relaxed atomic load when off.
//
// Three classes of finding, which are the three ways two threads can meet at one tile path:
//   concurrent writes   -- two threads publishing the same address at once (the fold's race)
//   read during write   -- a reader inside a writer's truncate/flush window (the torn read)
//   write during read   -- the same collision seen from the other side
// Plus GA_MAIN_THREAD_ONLY(), for the structures that carry no synchronization at all and are
// main-thread-only BY CONSTRUCTION today (PageTable, TileIndex, TileAtlas2D, every Gpu entry):
// an assumption nobody checks is a bug waiting for a thread manager to move the work.
#pragma once

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "Common.h"

namespace ga {
namespace threadaudit {

// The thread that owns the frame, the device, the queue and the command list. Stamped once at
// the top of main(), before any pool exists.
inline std::thread::id g_mainThread{};
inline std::atomic<bool> g_on{false};

inline void SetMainThread() { g_mainThread = std::this_thread::get_id(); }
inline bool OnMainThread() { return std::this_thread::get_id() == g_mainThread; }
inline void Enable() { g_on.store(true, std::memory_order_relaxed); }
inline bool On() { return g_on.load(std::memory_order_relaxed); }

struct Finding {
    std::string path;
    const char* kind;
};

struct State {
    std::mutex mx;
    // Per tile path: who is inside a write, and how many readers are inside a read.
    struct Live {
        int writers = 0;
        int readers = 0;
    };
    std::unordered_map<std::string, Live> live;
    uint64_t writes = 0, reads = 0, deletes = 0;
    uint64_t concurrentWrites = 0, readDuringWrite = 0, writeDuringRead = 0;
    uint64_t deleteDuringRead = 0, deleteDuringWrite = 0;
    std::vector<Finding> examples;   // first few, with their paths, for the report
    // Entry points that were declared main-thread-only and were not. Keyed by site so one
    // violation does not print ten thousand times.
    std::unordered_map<std::string, uint64_t> offMain;

    // The self-test stages real collisions on real files and must not leave them in the run's
    // report, so it snapshots the counters, zeroes them, and puts them back.
    struct Counts {
        uint64_t writes, reads, deletes;
        uint64_t concurrentWrites, readDuringWrite, writeDuringRead;
        uint64_t deleteDuringRead, deleteDuringWrite;
        std::vector<Finding> examples;
        std::unordered_map<std::string, uint64_t> offMain;
    };
    Counts Snapshot() const {
        return {writes,          reads,           deletes,          concurrentWrites,
                readDuringWrite, writeDuringRead, deleteDuringRead, deleteDuringWrite,
                examples,        offMain};
    }
    void ResetCounts() {
        writes = reads = deletes = 0;
        concurrentWrites = readDuringWrite = writeDuringRead = 0;
        deleteDuringRead = deleteDuringWrite = 0;
        examples.clear();
        offMain.clear();
    }
    void Restore(const Counts& c) {
        writes = c.writes;
        reads = c.reads;
        deletes = c.deletes;
        concurrentWrites = c.concurrentWrites;
        readDuringWrite = c.readDuringWrite;
        writeDuringRead = c.writeDuringRead;
        deleteDuringRead = c.deleteDuringRead;
        deleteDuringWrite = c.deleteDuringWrite;
        examples = c.examples;
        offMain = c.offMain;
    }
};

inline State& S() {
    static State s;
    return s;
}

inline void Note(State& s, const char* kind, const std::string& path) {
    if (s.examples.size() < 8) s.examples.push_back({path, kind});
}

// A write is open from the truncate to the flush -- exactly the window ReadTile can catch short.
struct WriteScope {
    std::string p;
    bool on;
    explicit WriteScope(const std::string& path) : p(path), on(On()) {
        if (!on) return;
        State& s = S();
        std::lock_guard<std::mutex> lk(s.mx);
        State::Live& l = s.live[p];
        ++s.writes;
        if (l.writers > 0) { ++s.concurrentWrites; Note(s, "concurrent-write", p); }
        if (l.readers > 0) { ++s.writeDuringRead; Note(s, "write-during-read", p); }
        ++l.writers;
    }
    ~WriteScope() {
        if (!on) return;
        State& s = S();
        std::lock_guard<std::mutex> lk(s.mx);
        auto it = s.live.find(p);
        if (it != s.live.end() && --it->second.writers <= 0 && it->second.readers <= 0) {
            s.live.erase(it);
        }
    }
    WriteScope(const WriteScope&) = delete;
    WriteScope& operator=(const WriteScope&) = delete;
};

struct ReadScope {
    std::string p;
    bool on;
    explicit ReadScope(const std::string& path) : p(path), on(On()) {
        if (!on) return;
        State& s = S();
        std::lock_guard<std::mutex> lk(s.mx);
        State::Live& l = s.live[p];
        ++s.reads;
        if (l.writers > 0) { ++s.readDuringWrite; Note(s, "read-during-write", p); }
        ++l.readers;
    }
    ~ReadScope() {
        if (!on) return;
        State& s = S();
        std::lock_guard<std::mutex> lk(s.mx);
        auto it = s.live.find(p);
        if (it != s.live.end() && --it->second.readers <= 0 && it->second.writers <= 0) {
            s.live.erase(it);
        }
    }
    ReadScope(const ReadScope&) = delete;
    ReadScope& operator=(const ReadScope&) = delete;
};

// DropCachedAddress deletes .bin/.void/base_* on the PARENT node under no lock; a delete that
// lands inside someone's read or write is the same collision as the two above.
inline void Deleted(const std::string& path) {
    if (!On()) return;
    State& s = S();
    std::lock_guard<std::mutex> lk(s.mx);
    ++s.deletes;
    auto it = s.live.find(path);
    if (it == s.live.end()) return;
    if (it->second.readers > 0) { ++s.deleteDuringRead; Note(s, "delete-during-read", path); }
    if (it->second.writers > 0) { ++s.deleteDuringWrite; Note(s, "delete-during-write", path); }
}

inline void OffMain(const char* site) {
    State& s = S();
    std::lock_guard<std::mutex> lk(s.mx);
    ++s.offMain[site];
}

inline void Report() {
    if (!On()) return;
    State& s = S();
    std::lock_guard<std::mutex> lk(s.mx);
    Log("[thread-audit] %llu tile writes, %llu reads, %llu deletes observed",
        static_cast<unsigned long long>(s.writes), static_cast<unsigned long long>(s.reads),
        static_cast<unsigned long long>(s.deletes));
    Log("[thread-audit]   concurrent writes  %llu   (two threads publishing one address)",
        static_cast<unsigned long long>(s.concurrentWrites));
    Log("[thread-audit]   read during write  %llu   (the torn read: ReadTile sees a short file)",
        static_cast<unsigned long long>(s.readDuringWrite));
    Log("[thread-audit]   write during read  %llu", static_cast<unsigned long long>(s.writeDuringRead));
    Log("[thread-audit]   delete during read %llu   write %llu",
        static_cast<unsigned long long>(s.deleteDuringRead),
        static_cast<unsigned long long>(s.deleteDuringWrite));
    for (const Finding& f : s.examples) Log("[thread-audit]     %-20s %s", f.kind, f.path.c_str());
    if (s.offMain.empty()) {
        Log("[thread-audit]   main-thread-only sites: no violations");
    } else {
        for (const auto& kv : s.offMain) {
            Log("[thread-audit]   OFF MAIN THREAD: %s x%llu", kv.first.c_str(),
                static_cast<unsigned long long>(kv.second));
        }
    }
}

}  // namespace threadaudit

// Defined in ThreadTest.cpp; runs under --selftest. The gate on the instrument above: it stages
// each collision deliberately and requires the audit to count exactly it.
bool RunThreadSelfTest();

}  // namespace ga

// Declares an entry point main-thread-only and CHECKS it under --thread-audit. Not an assert:
// RelWithDebInfo defines NDEBUG, and RelWithDebInfo is the build that actually runs here, so an
// assert would check nothing in the only configuration anyone measures.
#define GA_MAIN_THREAD_ONLY()                                            \
    do {                                                                 \
        if (::ga::threadaudit::On() && !::ga::threadaudit::OnMainThread()) { \
            ::ga::threadaudit::OffMain(__FUNCTION__);                    \
        }                                                                \
    } while (0)
