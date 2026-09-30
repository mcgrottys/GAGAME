// ================================================================================================
//  DayLedger - THE DAY'S CAP on what is ASKED of a paid source: at most `tiles` requests, and
//  `bytes` of tiles landed, in one UTC day, whichever is met first, over EVERY engine on the
//  machine together. The Google provider (core/TileProviders.h) holds one; the scene states the
//  caps (streaming.dayTiles, streaming.dayBytes). Zero is no request at all that day, never "no
//  limit".
//
//  THE LEDGER is one small file a day beside the provider's cache: <folder>\day_<YYYY-MM-DD>.json,
//      {"day": "2026-09-29", "asked": 1240, "tiles": 1234, "bytes": 45678901,
//       "last": "2026-09-29T14:03:12Z"}
//  `asked` is every request SENT, whatever came back; `tiles` and `bytes` are what LANDED whole;
//  `last` is when the last request was counted. It is written through a temporary file and a
//  rename, so a reader sees a whole file or the one before it. Old days' files stay where they
//  are: they are the history. The date is taken at each request, from the clock, so a run that
//  crosses midnight UTC begins the next day's file.
//
//  WHY `asked`: a source that answers every request with an error (an expired session, a quota)
//  lands nothing, so a ledger of landed tiles alone would never see it asked. The tiles' cap is
//  held on `asked` (asked >= tiles always), so it bounds what the engine ASKS, not what it got.
//
//  THE RULE (Fetch): under a named mutex every engine on the machine shares, read today's
//  ledger; if `asked` has reached the tiles' cap or `bytes` the bytes' cap, REFUSE. Otherwise
//  release the lock and make the request -- the lock is never held across the network. The
//  request answers one of three: NOT SENT (adds nothing), SENT AND FAILED (asked + 1), LANDED
//  whole (asked + 1, tiles + 1, bytes + its size). For the two that were sent, take the lock
//  again, READ THE LEDGER AGAIN (another engine may have written it while this one waited on the
//  network), add, and write.
//
//  WHAT IT DOES NOT PROMISE: a request is allowed on what the ledger held when it asked, so the
//  requests already past the check when a cap is met still go -- a day can close over its cap by
//  the requests in flight at that moment (one per fetching thread, over every engine), and the
//  bytes' cap by those requests' bytes, since a tile's size is known only once it arrives.
//
//  FAILING CLOSED: a ledger file that is there but cannot be read as a ledger (cut short,
//  hand-edited, without whole asked / tiles / bytes counts, fewer asked than tiles) is the cap
//  MET, not zero, and is said once a day. A lock that cannot be made refuses every request of the
//  run; a lock not had within kLockWaitMs refuses that request; a ledger that cannot be written
//  refuses every later request of the run. Each is logged.
//
//  The part that decides -- ledger, caps, lock, date -- is this class alone, so the selftest
//  ([daytest], DayLedgerTest.cpp) drives the engine's own rule with a stand-in for the request
//  and a stand-in for the clock, and never opens a connection.
// ================================================================================================
#pragma once

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace ga {

struct DayCaps {
    uint64_t tiles = 100000;           // streaming.dayTiles: requests a day (held on `asked`)
    uint64_t bytes = 5000000000ull;    // streaming.dayBytes: bytes landed a day
    // A scene number as a cap: negative or NaN is 0 (no request), past 2^64 is the most there is.
    static uint64_t FromScene(double v) {
        if (!(v > 0.0)) return 0;
        return v >= 1.8e19 ? UINT64_MAX : static_cast<uint64_t>(v);
    }
};

// What one day's ledger holds.
struct DayCount {
    uint64_t asked = 0;                // requests sent, whatever came back
    uint64_t tiles = 0, bytes = 0;     // what landed whole
    std::string last;                  // UTC time of the last request counted; empty = none yet
};

enum class DayRead { Missing, Whole, Damaged };      // Missing: a day with no request yet (zero)
enum class DaySent { No, Failed, Landed };           // a request's answer: not sent / sent and
                                                     // failed / sent and a whole body landed
enum class DayFetch { Fetched, Refused, NotSent, Failed };

class DayLedger {
public:
    using Clock = std::function<long long()>;                           // unix seconds, UTC
    using Request = std::function<DaySent(std::vector<uint8_t>& body)>;  // the network request
    static constexpr const wchar_t* kLock = L"Global\\GAGAME.dayLedger";
    static constexpr unsigned long kLockWaitMs = 10000;

    DayLedger() = default;
    DayLedger(const DayLedger&) = delete;
    DayLedger& operator=(const DayLedger&) = delete;
    ~DayLedger();

    // folder: where the day files live (the provider's cache folder). tag: the log lines' prefix.
    // clock: the selftest's stand-in; the engine passes none and the wall clock is read. Logs what
    // today's ledger holds and the caps. False when the lock cannot be made (every request refused).
    bool Open(const std::string& folder, const DayCaps& caps, const char* tag, Clock clock = {});

    // THE RULE. Not opened (the selftest's cache-only provider): the request, and nothing else.
    DayFetch Fetch(const Request& request, std::vector<uint8_t>& body);

    // The rule's two halves, public so a test can take them apart: Allow reads today's ledger
    // under the lock and says whether one more request may go (`seen`: what it read); Add reads
    // it AGAIN under the lock and writes it plus one asked -- and, when it landed, one tile and
    // `bytes`.
    bool Allow(DayCount* seen = nullptr);
    bool Add(bool landed, uint64_t bytes);
    // Writes `c` as the day's ledger, under the lock (the selftest's way to plant a state).
    bool Put(const std::string& day, const DayCount& c);

    std::string Today() const;                                 // "YYYY-MM-DD", UTC, from the clock
    std::string PathOf(const std::string& day) const;          // <folder>\day_<day>.json
    static DayRead Read(const std::string& path, DayCount& c, std::string* why = nullptr);
    // Times a lock was asked for while another holder had it (the selftest's evidence that two
    // writers really met at the lock).
    uint32_t Waits() const { return m_waits.load(std::memory_order_relaxed); }
    // Ledger reads the rule made (Allow and Add, each right after it took the lock; the boot
    // line's read is not counted): zero means the rule neither took the lock nor read the file.
    uint32_t Reads() const { return m_reads.load(std::memory_order_relaxed); }

private:
    bool Acquire();
    void Release();
    bool WriteLocked(const std::string& day, const DayCount& c);

    std::string m_folder, m_tag;
    DayCaps m_caps;
    Clock m_clock;
    void* m_lock = nullptr;            // HANDLE of the named mutex
    bool m_open = false;
    std::atomic<bool> m_broken{false};
    std::atomic<uint32_t> m_waits{0}, m_reads{0};
    std::string m_saidMet, m_saidDamaged;   // the day each was last logged; under the lock
    std::atomic<bool> m_saidLock{false};
};

bool RunDayLedgerSelfTest();   // DayLedgerTest.cpp: the [daytest] block of --selftest

}  // namespace ga
