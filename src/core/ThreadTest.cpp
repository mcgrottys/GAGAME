// RunThreadSelfTest -- the gate on the thread instrument itself.
//
// A clean [thread-audit] report is worth nothing unless the audit can see a collision when one
// is really there. This suite stages each of the four collisions deliberately, on real files in
// a scratch folder, from real threads, and requires the audit to count exactly them; then it
// requires GA_MAIN_THREAD_ONLY() to fire off the main thread and stay silent on it.
//
// It is the first gate of this work and the one that makes the others mean something: if this
// suite ever passes while the audit is blind, every later "0 collisions" is a lie.
#include <atomic>
#include <string>
#include <thread>
#include <vector>

#include "compose/TileTree.h"
#include "core/Common.h"
#include "core/ThreadAudit.h"

namespace ga {

namespace {

// A barrier for exactly two threads, so a collision is staged rather than hoped for. Two
// threads must be INSIDE their scopes at the same instant; sleeping and hoping is how a race
// test passes on a fast machine and proves nothing.
struct Gate {
    std::atomic<int> arrived{0};
    void Sync(int of = 2) {
        arrived.fetch_add(1, std::memory_order_acq_rel);
        while (arrived.load(std::memory_order_acquire) < of) std::this_thread::yield();
    }
};

bool Expect(const char* what, uint64_t got, uint64_t want) {
    if (got == want) return true;
    Log("[threadtest]   FAIL %s: %llu, expected %llu", what, static_cast<unsigned long long>(got),
        static_cast<unsigned long long>(want));
    return false;
}

}  // namespace

bool RunThreadSelfTest() {
    using namespace threadaudit;

    const bool wasOn = On();
    State& s = S();
    const State::Counts saved = s.Snapshot();   // the suite must not pollute the run's report

    Enable();
    s.ResetCounts();

    const std::string dir = "cache\\threadtest";
    tree_detail::MakeDir("cache");
    tree_detail::MakeDir(dir);
    const std::string path = dir + "\\gate.bin";
    const std::vector<uint8_t> bytes(65536, 0xA5);
    tree_detail::WriteTile(path, bytes);   // exists from here on
    s.ResetCounts();                       // ... and that write is not part of the count

    bool ok = true;

    // (1) two threads publishing ONE address at once -- the fold's race, and the one m_foldMx
    //     does not cover because LeafTile writes at TileTree.h:570 outside it.
    {
        Gate g;
        auto writer = [&] {
            WriteScope w(path);
            g.Sync();
            std::this_thread::yield();
        };
        std::thread a(writer), b(writer);
        a.join();
        b.join();
        ok &= Expect("concurrent writes", s.concurrentWrites, 1);
    }

    // (2) a reader inside a writer's truncate-to-flush window -- the torn read, which ReadTile
    //     reports as ABSENT and the caller then repaints over.
    //     The holder must still be INSIDE when the other arrives, so it waits on the other's
    //     completion rather than yielding and hoping: a barrier alone lets the first thread
    //     leave its scope before the second enters, and the collision never happens. (It did
    //     not, the first time this was written, and this suite is what said so.)
    s.ResetCounts();
    {
        std::atomic<bool> writerIn{false}, readerDone{false};
        std::thread w([&] {
            WriteScope ws(path);
            writerIn.store(true, std::memory_order_release);
            while (!readerDone.load(std::memory_order_acquire)) std::this_thread::yield();
        });
        std::thread r([&] {
            while (!writerIn.load(std::memory_order_acquire)) std::this_thread::yield();
            { ReadScope rs(path); }   // entered with the writer provably still inside
            readerDone.store(true, std::memory_order_release);
        });
        w.join();
        r.join();
        ok &= Expect("read during write", s.readDuringWrite, 1);
    }

    // (3) a delete landing inside a read -- DropCachedAddress unlinks .bin/.void on the PARENT
    //     node under no lock at all (TileTree.h:824-839).
    s.ResetCounts();
    {
        std::atomic<bool> readerIn{false}, deleteDone{false};
        std::thread r([&] {
            ReadScope rs(path);
            readerIn.store(true, std::memory_order_release);
            while (!deleteDone.load(std::memory_order_acquire)) std::this_thread::yield();
        });
        std::thread d([&] {
            while (!readerIn.load(std::memory_order_acquire)) std::this_thread::yield();
            Deleted(path);
            deleteDone.store(true, std::memory_order_release);
        });
        r.join();
        d.join();
        ok &= Expect("delete during read", s.deleteDuringRead, 1);
    }

    // (4) the quiet case: sequential access to one path is NOT a finding. An instrument that
    //     cries wolf on the normal path would make every later report unreadable.
    s.ResetCounts();
    {
        { WriteScope w(path); }
        { ReadScope r(path); }
        Deleted(path);
        ok &= Expect("collisions when serial", s.concurrentWrites + s.readDuringWrite +
                                                   s.writeDuringRead + s.deleteDuringRead +
                                                   s.deleteDuringWrite,
                     0);
    }

    // (5) GA_MAIN_THREAD_ONLY: silent on the frame thread, loud off it.
    s.ResetCounts();
    {
        GA_MAIN_THREAD_ONLY();
        ok &= Expect("off-main sites from the main thread", s.offMain.size(), 0);
        std::thread t([] { GA_MAIN_THREAD_ONLY(); });
        t.join();
        ok &= Expect("off-main sites from a worker", s.offMain.size(), 1);
    }

    // (6) THE ONE THAT MATTERS: a reader must never see a tile that is neither the old one nor
    //     the new one. This drives the REAL tree_detail::WriteTile and ReadTile, not the audit,
    //     because the audit only says two threads met -- it cannot say what the reader got.
    //
    //     A truncating publish fails here, and must: ReadTile's size check catches the short
    //     file and returns false, which every caller in TileTree.h reads as ABSENT -- Serve
    //     falls through to a repaint and FoldUp repaints the parent from the source, throwing
    //     away the folds already in it. `absent` below counts exactly those.
    //
    //     Worth recording why this test exists at all: the storm rail with a warm tree cache
    //     does 386 tile writes and 27732 reads and meets ZERO collisions, so no rail gates this.
    //     The race needs a workload that paints, and this is the smallest honest one.
    {
        const std::string rw = dir + "\\publish.bin";
        std::vector<uint8_t> a(65536, 0xA1), b(65536, 0xB2);
        tree_detail::WriteTile(rw, a);
        std::atomic<bool> stop{false};
        std::atomic<uint32_t> absent{0}, torn{0}, whole{0}, lost{0};
        std::thread w([&] {
            for (int i = 0; i < 400; ++i) {
                if (!tree_detail::WriteTile(rw, (i & 1) ? b : a)) {
                    lost.fetch_add(1, std::memory_order_relaxed);
                }
            }
            stop.store(true, std::memory_order_release);
        });
        std::thread r([&] {
            std::vector<uint8_t> got;
            while (!stop.load(std::memory_order_acquire)) {
                if (!tree_detail::ReadTile(rw, got)) {
                    absent.fetch_add(1, std::memory_order_relaxed);
                    continue;
                }
                const uint8_t v = got[0];
                bool uniform = (v == 0xA1 || v == 0xB2);
                for (size_t k = 0; uniform && k < got.size(); ++k) uniform = got[k] == v;
                (uniform ? whole : torn).fetch_add(1, std::memory_order_relaxed);
            }
        });
        w.join();
        r.join();
        Log("[threadtest]   publish/read race over 400 publishes: %u whole reads, %u ABSENT, "
            "%u TORN, %u publishes LOST",
            whole.load(), absent.load(), torn.load(), lost.load());
        ok &= Expect("reads that saw no tile while one was always there", absent.load(), 0);
        ok &= Expect("reads that saw a mixture of two tiles", torn.load(), 0);
        // A publish a concurrent reader makes FAIL is the same lost tile by another route --
        // MoveFileEx over a file someone holds open is the way this fix could pay for itself
        // with a new defect, so it is gated, not assumed.
        ok &= Expect("publishes lost to a concurrent reader", lost.load(), 0);
        tree_detail::Delete(rw);
    }

    tree_detail::Delete(path);
    s.ResetCounts();
    s.Restore(saved);
    if (!wasOn) g_on.store(false, std::memory_order_relaxed);

    Log("[threadtest] ---- %s: the audit sees concurrent writes, torn reads, deletes under a "
        "reader, stays quiet when serial, and GA_MAIN_THREAD_ONLY fires only off the frame "
        "thread ----",
        ok ? "PASS" : "FAIL");
    return ok;
}

}  // namespace ga
