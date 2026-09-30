// ================================================================================================
//  DayLedgerTest - the [daytest] block of --selftest: THE DAY'S CAP on what the engine asks of
//  Google (core/DayLedger.h), the engine's own rule driven with a stand-in for the request and a
//  stand-in for the clock -- and the Google provider's own rule, row of failures and budget,
//  driven through its cache-only form with the same stand-ins. No connection is ever opened.
//
//  What is real: the DayLedger class the provider holds, its ledger files, its caps, its date and
//  the named mutex every engine on the machine shares (the test takes the engines' own lock, so
//  it also shows this process can make it); the provider's FetchTile, budget, refusals and row of
//  failures. What stands in: the request -- a lambda that hands back N bytes, fails, is never
//  sent, or runs another writer's whole fetch while it "waits on the network" -- and the clock, a
//  number the test sets. Http's whole-body rule is NOT here: it needs a connection, and no server
//  is invented for it.
//
//  THE SCRATCH is out\daytest\<UTC stamp>_<pid>\, one folder per case; nothing touches the cache.
//  When every check has passed the test deletes the files it made, folder by folder (no
//  recursion), then the folders; when one has failed, the scratch is left and named.
//
//  THE PLANTED FAILURES, each of which the test must see fail: the rule with the lock's re-read
//  taken out (two writers interleaved across a request lose a count under it); the old rule that
//  counted only what landed (five failures let a sixth request go under it); the old log form of
//  a request (the first 60 characters, caught holding a MADE-UP key). The real key is never read.
// ================================================================================================
#include "core/DayLedger.h"
#include "core/TileProviders.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <atomic>
#include <condition_variable>
#include <cstdarg>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "core/Common.h"

namespace ga {

namespace {

bool g_ok = true;

void Check(bool cond, const std::string& what) {
    if (cond) {
        Log("[daytest] %s", what.c_str());
    } else {
        Log("[daytest]   FAIL: %s", what.c_str());
        g_ok = false;
    }
}
// A planted failure: the test must SEE it fail.
void Caught(bool cond, const std::string& what) {
    if (cond) {
        Log("[daytest] CAUGHT: %s", what.c_str());
    } else {
        Log("[daytest]   FAIL (MISSED): %s", what.c_str());
        g_ok = false;
    }
}

std::string Fmt(const char* fmt, ...) {
    char b[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    return b;
}

unsigned long long U(uint64_t v) { return static_cast<unsigned long long>(v); }
const char* Said(DayFetch f) {
    return f == DayFetch::Fetched   ? "fetched"
           : f == DayFetch::Refused ? "REFUSED"
           : f == DayFetch::NotSent ? "not sent"
                                    : "failed";
}

long long Utc(int y, int mo, int d, int h, int mi, int s) {
    std::tm t{};
    t.tm_year = y - 1900;
    t.tm_mon = mo - 1;
    t.tm_mday = d;
    t.tm_hour = h;
    t.tm_min = mi;
    t.tm_sec = s;
    return static_cast<long long>(_mkgmtime(&t));
}

std::string Slurp(const std::string& p) {
    std::ifstream f(p, std::ios::binary);
    return std::string((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}
void Plant(const std::string& p, const std::string& text) {
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    f.write(text.data(), static_cast<std::streamsize>(text.size()));
}
bool Exists(const std::string& p) { return GetFileAttributesA(p.c_str()) != INVALID_FILE_ATTRIBUTES; }

// Where the brief says a day's ledger lives, spelled out here so the test also holds the name.
std::string Path(const std::string& dir, const std::string& day) {
    return dir + "\\day_" + day + ".json";
}
DayCount Ledger(const std::string& dir, const std::string& day, DayRead* r = nullptr) {
    DayCount c;
    const DayRead got = DayLedger::Read(Path(dir, day), c);
    if (r) *r = got;
    return c;
}

// The stand-in request: hands back `n` bytes, fails, or is never sent; counts its calls.
struct Stand {
    std::atomic<int> calls{0};
    DayLedger::Request Bytes(size_t n) {
        return [this, n](std::vector<uint8_t>& b) {
            calls.fetch_add(1);
            b.assign(n, 0x5a);
            return DaySent::Landed;
        };
    }
    DayLedger::Request Fail() {
        return [this](std::vector<uint8_t>&) {
            calls.fetch_add(1);
            return DaySent::Failed;
        };
    }
    DayLedger::Request NotSent() {
        return [this](std::vector<uint8_t>&) {
            calls.fetch_add(1);
            return DaySent::No;
        };
    }
};

// A latch the test opens: a stand-in request waits on it, so the test decides when "the network"
// answers -- after every thread it started is inside the provider.
struct Latch {
    std::mutex mx;
    std::condition_variable cv;
    bool open = false;
    void Wait() {
        std::unique_lock<std::mutex> lk(mx);
        cv.wait(lk, [this] { return open; });
    }
    void Open() {
        {
            std::lock_guard<std::mutex> lk(mx);
            open = true;
        }
        cv.notify_all();
    }
};
// Waits up to five seconds for `pred`; says whether it came true.
template <class Pred>
bool Until(Pred pred) {
    for (int i = 0; i < 5000 && !pred(); ++i) Sleep(1);
    return pred();
}

}  // namespace

bool RunDayLedgerSelfTest() {
    g_ok = true;
    Log("[daytest] ---- the day's cap on what is asked of Google: the engine's ledger, caps, lock "
        "and date, and the provider's rule, row and budget; a stand-in request and clock; scratch "
        "under out\\daytest; no network ----");

    // ---- where: the worktree's own out\, a plain folder -- or nothing is written at all ----------
    char cwd[MAX_PATH];
    const DWORD cwdLen = GetCurrentDirectoryA(MAX_PATH, cwd);
    if (cwdLen == 0 || cwdLen >= MAX_PATH) {
        Log("[daytest]   FAIL: the working folder cannot be read, so the test writes nothing");
        return false;
    }
    const std::string out = std::string(cwd, cwdLen) + "\\out", base = out + "\\daytest";
    CreateDirectoryA(out.c_str(), nullptr);
    CreateDirectoryA(base.c_str(), nullptr);
    for (const std::string& d : {out, base}) {
        const DWORD a = GetFileAttributesA(d.c_str());
        if (a == INVALID_FILE_ATTRIBUTES || !(a & FILE_ATTRIBUTE_DIRECTORY) ||
            (a & FILE_ATTRIBUTE_REPARSE_POINT)) {
            Log("[daytest]   FAIL: %s is not a plain folder, so the test writes nothing", d.c_str());
            return false;
        }
    }
    const time_t wall = time(nullptr);
    std::tm wt{};
    gmtime_s(&wt, &wall);
    char stamp[32];
    strftime(stamp, sizeof stamp, "%Y%m%dT%H%M%SZ", &wt);
    const std::string run = base + "\\" + stamp + "_" + std::to_string(GetCurrentProcessId());
    if (!CreateDirectoryA(run.c_str(), nullptr)) {
        Log("[daytest]   FAIL: cannot make a new scratch folder %s", run.c_str());
        return false;
    }
    std::vector<std::string> dirs;   // every case's folder, emptied and removed at the end
    auto caseDir = [&](const char* name) {
        const std::string d = run + "\\" + name;
        CreateDirectoryA(d.c_str(), nullptr);
        dirs.push_back(d);
        return d;
    };

    long long now = Utc(2026, 9, 29, 12, 0, 0);   // THE STAND-IN CLOCK: the test moves it
    const DayLedger::Clock clock = [&now] { return now; };
    const std::string day1 = "2026-09-29", day2 = "2026-09-30";
    std::vector<uint8_t> body;

    // ---- 1. the count rises to the cap, and the next fetch is refused -------------------------
    {
        const std::string dir = caseDir("count");
        DayLedger L;
        L.Open(dir, DayCaps{5, 1000000}, "[daytest]", clock);
        Stand s;
        std::string rise;
        bool rose = true;
        for (uint64_t i = 1; i <= 5; ++i) {
            const DayFetch f = L.Fetch(s.Bytes(100), body);
            const DayCount c = Ledger(dir, day1);
            rose &= f == DayFetch::Fetched && c.asked == i && c.tiles == i && c.bytes == 100 * i;
            rise += (i > 1 ? " " : "") + std::to_string(c.tiles);
        }
        const DayFetch sixth = L.Fetch(s.Bytes(100), body);
        const DayCount c = Ledger(dir, day1);
        Check(rose && sixth == DayFetch::Refused && s.calls == 5 && c.asked == 5 && c.tiles == 5 &&
                  c.bytes == 500,
              Fmt("the count rises to the cap: at 5 a day the ledger reads %s tiles after each "
                  "fetch; the 6th is %s, the request was made %d times, the ledger still %llu "
                  "asked, %llu tiles, %llu bytes",
                  rise.c_str(), Said(sixth), s.calls.load(), U(c.asked), U(c.tiles), U(c.bytes)));
    }

    // ---- 2. the bytes' cap met before the tiles' ----------------------------------------------
    {
        const std::string dir = caseDir("bytes");
        DayLedger L;
        L.Open(dir, DayCaps{100, 1000}, "[daytest]", clock);
        Stand s;
        int fetched = 0;
        DayFetch f = DayFetch::Fetched;
        while (fetched < 10 && (f = L.Fetch(s.Bytes(400), body)) == DayFetch::Fetched) ++fetched;
        const DayCount c = Ledger(dir, day1);
        Check(fetched == 3 && f == DayFetch::Refused && s.calls == 3 && c.tiles == 3 &&
                  c.bytes == 1200,
              Fmt("the bytes' cap first: at 1000 bytes and 100 requests a day, 400-byte tiles go "
                  "%d times (the ledger %llu tiles, %llu bytes: the 3rd was asked at 800, and a "
                  "tile's size is known only when it lands) and the next is %s with the tiles' "
                  "cap %llu away",
                  fetched, U(c.tiles), U(c.bytes), Said(f), U(100 - c.asked)));
    }

    // ---- 3. a cap of zero refuses the first fetch ---------------------------------------------
    {
        const std::string dir = caseDir("zero");
        Stand s;
        DayLedger A, B;
        A.Open(dir, DayCaps{0, 5000000000ull}, "[daytest]", clock);
        const DayFetch a = A.Fetch(s.Bytes(100), body);
        B.Open(dir, DayCaps{100000, 0}, "[daytest]", clock);
        const DayFetch b = B.Fetch(s.Bytes(100), body);
        Check(a == DayFetch::Refused && b == DayFetch::Refused && s.calls == 0 &&
                  !Exists(Path(dir, day1)),
              Fmt("a cap of zero is no request, not no limit: with dayTiles 0 the first fetch is "
                  "%s, with dayBytes 0 it is %s; the request was made %d times and no ledger "
                  "was written",
                  Said(a), Said(b), s.calls.load()));
    }

    // ---- 4. the day turns while the engine runs -----------------------------------------------
    {
        now = Utc(2026, 9, 29, 23, 59, 50);
        const std::string dir = caseDir("turn");
        DayLedger L;
        L.Open(dir, DayCaps{2, 1000000}, "[daytest]", clock);
        Stand s;
        const DayFetch f1 = L.Fetch(s.Bytes(300), body), f2 = L.Fetch(s.Bytes(300), body);
        const DayFetch f3 = L.Fetch(s.Bytes(300), body);
        const std::string old = Slurp(Path(dir, day1));
        now = Utc(2026, 9, 30, 0, 0, 5);   // midnight UTC passes; nothing is re-opened
        const DayFetch f4 = L.Fetch(s.Bytes(700), body);
        const DayCount c = Ledger(dir, day2), o = Ledger(dir, day1);
        Check(f1 == DayFetch::Fetched && f2 == DayFetch::Fetched && f3 == DayFetch::Refused &&
                  f4 == DayFetch::Fetched && c.asked == 1 && c.tiles == 1 && c.bytes == 700 &&
                  c.last == "2026-09-30T00:00:05Z" && o.tiles == 2 && !old.empty() &&
                  Slurp(Path(dir, day1)) == old,
              Fmt("the day turns at a fetch, not at boot: 2 tiles fill 2026-09-29 by 23:59:50 and "
                  "the 3rd is %s; at 00:00:05 the next is %s, day_2026-09-30.json begins at %llu "
                  "asked, %llu tile, %llu bytes (last %s), and day_2026-09-29.json is byte for "
                  "byte as it was (%llu tiles, %llu bytes)",
                  Said(f3), Said(f4), U(c.asked), U(c.tiles), U(c.bytes), c.last.c_str(),
                  U(o.tiles), U(o.bytes)));
        now = Utc(2026, 9, 29, 12, 0, 0);
    }

    // ---- 5a. two writers interleaved across a request -----------------------------------------
    // Two ledgers on one folder, as two engines are. Writer 2 asks, and while it "waits on the
    // network" writer 1 asks, fetches and writes, whole; then writer 2 writes.
    {
        const std::string dir = caseDir("interleave");
        DayLedger L1, L2;
        L1.Open(dir, DayCaps{100, 1000000}, "[daytest]", clock);
        L2.Open(dir, DayCaps{100, 1000000}, "[daytest]", clock);
        Stand s;
        DayFetch first = DayFetch::Failed;
        uint64_t asked = 99;
        const DayFetch second = L2.Fetch(
            [&](std::vector<uint8_t>& b) {
                asked = Ledger(dir, day1).asked;   // what writer 2's check just read
                std::vector<uint8_t> b1;
                first = L1.Fetch(s.Bytes(1000), b1);
                b.assign(2000, 0x5a);
                return DaySent::Landed;
            },
            body);
        const DayCount c = Ledger(dir, day1);
        Check(first == DayFetch::Fetched && second == DayFetch::Fetched && asked == 0 &&
                  c.asked == 2 && c.tiles == 2 && c.bytes == 3000,
              Fmt("two writers interleaved: writer 2 asked at %llu; writer 1 fetched 1000 bytes and "
                  "wrote during writer 2's request; writer 2 then wrote its 2000 -- the ledger "
                  "reads %llu asked, %llu tiles, %llu bytes: neither count lost, because the "
                  "write re-reads under the lock",
                  U(asked), U(c.asked), U(c.tiles), U(c.bytes)));
    }

    // ---- 5b. two writers on two threads, meeting at the lock ----------------------------------
    {
        const std::string dir = caseDir("threads");
        DayLedger L1, L2;
        L1.Open(dir, DayCaps{100000, 5000000000ull}, "[daytest]", clock);
        L2.Open(dir, DayCaps{100000, 5000000000ull}, "[daytest]", clock);
        constexpr int kN = 150;
        auto size = [](int i, int salt) { return static_cast<size_t>(10 * (1 + (i + salt) % 7)); };
        auto writer = [&](DayLedger& L, int salt) {
            std::vector<uint8_t> b;
            for (int i = 0; i < kN; ++i) {
                L.Fetch(
                    [&](std::vector<uint8_t>& o) {
                        std::this_thread::yield();   // the "network": the other writer runs
                        o.assign(size(i, salt), 0x5a);
                        return DaySent::Landed;
                    },
                    b);
            }
        };
        uint64_t want = 0;
        for (int i = 0; i < kN; ++i) want += size(i, 0) + size(i, 3);
        std::thread t1([&] { writer(L1, 0); }), t2([&] { writer(L2, 3); });
        t1.join();
        t2.join();
        const DayCount c = Ledger(dir, day1);
        const uint32_t waits = L1.Waits() + L2.Waits();
        Check(c.asked == 2 * kN && c.tiles == 2 * kN && c.bytes == want && waits > 0,
              Fmt("two writers on two threads, %d fetches each through their own handles on the "
                  "one named lock: the ledger reads %llu asked, %llu tiles, %llu bytes (%d and "
                  "%llu made) -- none lost; a writer found the lock held %u times",
                  kN, U(c.asked), U(c.tiles), U(c.bytes), 2 * kN, U(want), waits));
    }

    // ---- 6. a damaged ledger is the cap met, not zero -----------------------------------------
    {
        const std::string dir = caseDir("damaged");
        DayLedger L;
        L.Open(dir, DayCaps{100, 1000000}, "[daytest]", clock);
        Stand s;
        for (int i = 0; i < 3; ++i) L.Fetch(s.Bytes(250), body);
        const std::string path = Path(dir, day1), whole = Slurp(path);
        const std::string cut = whole.substr(0, whole.size() / 2);
        Plant(path, cut);
        DayRead r = DayRead::Whole;
        Ledger(dir, day1, &r);
        const DayFetch f = L.Fetch(s.Bytes(250), body);   // the DAMAGED line prints just above
        Check(!whole.empty() && r == DayRead::Damaged && f == DayFetch::Refused && s.calls == 3 &&
                  Slurp(path) == cut,
              Fmt("a ledger cut short (%zu of its %zu bytes) is the cap MET, not zero: the next "
                  "fetch is %s, the request was made %d times (the 3 before the cut), the file "
                  "is left as it was, and the engine says so (the DAMAGED line above)",
                  cut.size(), whole.size(), Said(f), s.calls.load()));
    }

    // ---- 7. PLANTED: the rule with the lock's re-read taken out -------------------------------
    // Writer 2 under the planted rule: ask, the request (writer 1 whole inside it, as in 5a),
    // then the count read BEFORE the request plus one, written back without reading again.
    {
        const std::string dir = caseDir("planted");
        DayLedger L1, L2;
        L1.Open(dir, DayCaps{100, 1000000}, "[daytest]", clock);
        L2.Open(dir, DayCaps{100, 1000000}, "[daytest]", clock);
        Stand s;
        DayCount seen;
        const bool allowed = L2.Allow(&seen);
        std::vector<uint8_t> b1;
        const DayFetch first = L1.Fetch(s.Bytes(1000), b1);   // during writer 2's "request"
        DayCount stale = seen;
        stale.asked += 1;
        stale.tiles += 1;
        stale.bytes += 2000;
        stale.last = "2026-09-29T12:00:00Z";
        const bool put = allowed && L2.Put(L2.Today(), stale);
        const DayCount c = Ledger(dir, day1);
        Caught(put && first == DayFetch::Fetched && c.asked == 1 && c.tiles == 1 && c.bytes == 2000,
               Fmt("PLANTED, the rule with the lock's re-read taken out, the interleave of 5a: the "
                   "ledger reads %llu asked, %llu tile, %llu bytes -- writer 1's fetch is LOST, "
                   "where the engine's rule kept both",
                   U(c.asked), U(c.tiles), U(c.bytes)));
    }

    // ---- 8. a request that fails is asked, not a tile -----------------------------------------
    {
        const std::string dir = caseDir("failed");
        DayLedger L;
        L.Open(dir, DayCaps{5, 1000000}, "[daytest]", clock);
        Stand s;
        int failed = 0;
        for (int i = 0; i < 5; ++i) failed += L.Fetch(s.Fail(), body) == DayFetch::Failed ? 1 : 0;
        const DayFetch sixth = L.Fetch(s.Fail(), body);
        const DayCount c = Ledger(dir, day1);
        Check(failed == 5 && sixth == DayFetch::Refused && s.calls == 5 && c.asked == 5 &&
                  c.tiles == 0 && c.bytes == 0,
              Fmt("a request that fails is asked, not a tile: at dayTiles 5, %d failures close the "
                  "day with the ledger at %llu asked, %llu tiles, %llu bytes, and the 6th is %s -- "
                  "the request made %d times",
                  failed, U(c.asked), U(c.tiles), U(c.bytes), Said(sixth), s.calls.load()));
    }

    // ---- 9. a request not sent adds nothing ---------------------------------------------------
    {
        const std::string dir = caseDir("notsent");
        DayLedger L;
        L.Open(dir, DayCaps{5, 1000000}, "[daytest]", clock);
        Stand s;
        const DayFetch f = L.Fetch(s.NotSent(), body);
        DayRead r = DayRead::Whole;
        const DayCount c = Ledger(dir, day1, &r);
        Check(f == DayFetch::NotSent && s.calls == 1 && r == DayRead::Missing && c.asked == 0,
              Fmt("a request not sent adds nothing: the rule answers %s and today's ledger is %s "
                  "(%llu asked)",
                  Said(f), r == DayRead::Missing ? "not written" : "WRITTEN", U(c.asked)));
    }

    // ---- 10. a ledger without `asked` is damaged, the cap met ---------------------------------
    {
        const std::string dir = caseDir("noasked");
        DayLedger L;
        L.Open(dir, DayCaps{100, 1000000}, "[daytest]", clock);
        const std::string firstCut =   // the form this ledger's first cut wrote
            "{\"day\": \"2026-09-29\", \"tiles\": 3, \"bytes\": 750, \"last\": "
            "\"2026-09-29T12:00:00Z\"}\n";
        Plant(Path(dir, day1), firstCut);
        DayRead r = DayRead::Whole;
        Ledger(dir, day1, &r);
        Stand s;
        const DayFetch f = L.Fetch(s.Bytes(100), body);
        Check(r == DayRead::Damaged && f == DayFetch::Refused && s.calls == 0 &&
                  Slurp(Path(dir, day1)) == firstCut,
              Fmt("a ledger without `asked` (day, tiles, bytes, last: the first cut's form) is "
                  "DAMAGED, the cap MET: the next fetch is %s, the request made %d times, the "
                  "file left as it was",
                  Said(f), s.calls.load()));
    }

    // ---- 11. the row of failures: the provider's own rule stops asking ------------------------
    // The provider's cache-only form with a day ledger in the case folder and a stand-in for the
    // network; Fetch() is its FetchTile without the decode, so nothing here starts COM.
    {
        const DayCaps caps{100, 1000000};
        std::vector<uint8_t> jpg;
        const std::string dirP = caseDir("row");
        int sentP = 0;
        GoogleTileProvider P;
        P.InitCacheOnly(dirP, "satellite", 100, &caps, clock, [&sentP](std::vector<uint8_t>&) {
            ++sentP;
            return DaySent::Failed;
        });
        int gotP = 0;
        for (int i = 0; i < 12; ++i) gotP += P.Fetch(12, 100 + i, 200, jpg) ? 1 : 0;
        const DayCount cp = Ledger(dirP, day1);
        Check(sentP == 8 && gotP == 0 && P.Refused() == 4 && cp.asked == 8 && cp.tiles == 0 &&
                  P.DayReads() == 16,
              Fmt("the row of failures, the provider's own: 12 tiles asked of a source that only "
                  "errs -- %d requests sent, the row stops the run at %u (said once, above), and "
                  "the %u after it are REFUSED unsent; the ledger reads %llu asked, %llu tiles, "
                  "and the rule read it %u times",
                  sentP, GoogleTileProvider::kFailRow, P.Refused(), U(cp.asked), U(cp.tiles),
                  P.DayReads()));

        const std::string dirQ = caseDir("reset");
        int sentQ = 0;
        GoogleTileProvider Q;
        Q.InitCacheOnly(dirQ, "satellite", 100, &caps, clock, [&sentQ](std::vector<uint8_t>& b) {
            ++sentQ;
            if (sentQ != 8) return DaySent::Failed;
            b.assign(300, 0x5a);
            return DaySent::Landed;
        });
        int landed = 0;
        for (int i = 0; i < 15; ++i) landed += Q.Fetch(12, 100 + i, 300, jpg) ? 1 : 0;
        const uint32_t refused15 = Q.Refused();
        Q.Fetch(12, 200, 300, jpg);   // the 8th failure in a row since the landed tile
        Q.Fetch(12, 201, 300, jpg);   // after the stop
        const DayCount cq = Ledger(dirQ, day1);
        Check(landed == 1 && refused15 == 0 && sentQ == 16 && Q.Refused() == 1 && cq.asked == 16 &&
                  cq.tiles == 1 && cq.bytes == 300,
              Fmt("a landed tile resets the row: 7 fail, 1 lands, 7 fail and %u are refused; the "
                  "next failure is the 8th in a row and stops the run, and the one after is "
                  "REFUSED unsent (%d sent, %u refused; the ledger %llu asked, %llu tile, %llu "
                  "bytes)",
                  refused15, sentQ, Q.Refused(), U(cq.asked), U(cq.tiles), U(cq.bytes)));
    }

    // ---- 12. the budget is asked before the ledger --------------------------------------------
    {
        const DayCaps caps{100, 1000000};
        const std::string dir = caseDir("budget");
        int sent = 0;
        GoogleTileProvider B;
        B.InitCacheOnly(dir, "satellite", 0, &caps, clock, [&sent](std::vector<uint8_t>&) {
            ++sent;
            return DaySent::Failed;
        });
        std::vector<uint8_t> jpg;
        int got = 0;
        for (int i = 0; i < 5; ++i) got += B.Fetch(12, 100 + i, 500, jpg) ? 1 : 0;
        Check(got == 0 && B.Refused() == 5 && B.DayReads() == 0 && sent == 0 &&
                  !Exists(Path(dir, day1)),
              Fmt("budget first: at tileBudget 0 the provider REFUSES %u of 5 fetches with %u "
                  "ledger reads -- the machine's lock not taken, the file neither read nor "
                  "written, the request made %d times",
                  B.Refused(), B.DayReads(), sent));
    }

    // ---- 13. PLANTED: the old rule, counting only what landed ---------------------------------
    {
        const std::string dir = caseDir("oldrule");
        DayLedger L;
        L.Open(dir, DayCaps{5, 1000000}, "[daytest]", clock);
        Stand s;
        auto oldRule = [&L](const DayLedger::Request& r) {
            if (!L.Allow()) return DayFetch::Refused;
            std::vector<uint8_t> b;
            if (r(b) != DaySent::Landed) return DayFetch::Failed;   // THE PLANT: adds nothing
            L.Add(true, b.size());
            return DayFetch::Fetched;
        };
        for (int i = 0; i < 5; ++i) oldRule(s.Fail());
        const DayFetch sixth = oldRule(s.Fail());
        const DayCount c = Ledger(dir, day1);
        Caught(sixth != DayFetch::Refused && s.calls == 6 && c.asked == 0,
               Fmt("PLANTED, the old rule (count only what landed) against five failures: the "
                   "ledger reads %llu asked and the 6th is %s -- the request made %d times, a "
                   "source that only errs asked on and on",
                   U(c.asked), Said(sixth), s.calls.load()));
    }

    // ---- 14. what a log line may say of a request: the path before the '?' --------------------
    // MADE-UP secrets: a key of 39 characters (a Google key's length) and a token, neither with a
    // real key's prefix, so no secret scanner takes this file for a leak. The real key is never
    // read, and these are never printed: a line says how much of one it holds, not what.
    {
        const std::string fakeKey = "FAKEKEY-daytest-0123456789abcdefghijklm";
        const std::string fakeToken = "FAKETOKEN-daytest-9876543210zyxwvu";
        auto wide = [](const std::string& s) { return std::wstring(s.begin(), s.end()); };
        const std::wstring session = L"/v1/createSession?key=" + wide(fakeKey);
        const std::wstring tile =
            L"/v1/2dtiles/14/4953/6059?session=" + wide(fakeToken) + L"&key=" + wide(fakeKey);
        // Holds any 8-character piece of the secret, anywhere.
        auto holds = [](const std::string& s, const std::string& secret) {
            for (size_t i = 0; i + 8 <= secret.size(); ++i) {
                if (s.find(secret.substr(i, 8)) != std::string::npos) return true;
            }
            return false;
        };
        const std::string s1 = LoggablePath(session), s2 = LoggablePath(tile);
        Check(s1 == "/v1/createSession" && s2 == "/v1/2dtiles/14/4953/6059" &&
                  !holds(s1, fakeKey) && !holds(s2, fakeKey) && !holds(s2, fakeToken),
              Fmt("a request as a log line names it: \"%s\" and \"%s\" -- the path before the "
                  "'?', holding no 8-character piece of the made-up key or token",
                  s1.c_str(), s2.c_str()));
        // PLANTED: the old form, the path's first 60 characters, which the session's request
        // filled with its key.
        std::string old;
        for (const wchar_t c : session.substr(0, 60)) old += static_cast<char>(c);   // ASCII here
        size_t held = 0;
        while (held < fakeKey.size() && old.find(fakeKey.substr(0, held + 1)) != std::string::npos) {
            ++held;
        }
        Caught(holds(old, fakeKey) && held + 1 == fakeKey.size(),
               Fmt("PLANTED, the old form (the path's first 60 characters): it holds the made-up "
                   "key's first %zu of %zu characters",
                   held, fakeKey.size()));
    }

    // ---- 15-18. ONE THREAD AT A TIME FETCHES A TILE: the provider's flight --------------------
    // The stand-in request waits on a latch the test opens once every thread it started is inside
    // the provider -- one in "the network", the others waiting on that fetch. Tiles land in the
    // case folder's satellite\ (made here), where a thread that waited takes its result.
    {
        const DayCaps caps{100, 1000000};
        auto flightCase = [&](const char* name) {
            const std::string dir = caseDir(name), sat = dir + "\\satellite";
            CreateDirectoryA(sat.c_str(), nullptr);
            dirs.push_back(sat);
            return dir;
        };
        auto lands = [](Latch& latch, std::atomic<int>& sent, uint8_t fill) {
            return [&latch, &sent, fill](std::vector<uint8_t>& b) {
                sent.fetch_add(1);
                latch.Wait();
                b.assign(300, fill);
                return DaySent::Landed;
            };
        };
        // 15. two threads, one tile.
        {
            const std::string dir = flightCase("flight");
            Latch latch;
            std::atomic<int> sent{0};
            GoogleTileProvider P;
            P.InitCacheOnly(dir, "satellite", 100, &caps, clock, lands(latch, sent, 0x31));
            std::vector<uint8_t> j1, j2;
            bool r1 = false, r2 = false;
            std::thread t1([&] { r1 = P.Fetch(13, 2000, 3000, j1); });
            Until([&] { return sent.load() == 1; });
            std::thread t2([&] { r2 = P.Fetch(13, 2000, 3000, j2); });
            const bool inside = Until([&] { return P.FlightWaits() >= 1; });
            latch.Open();
            t1.join();
            t2.join();
            const DayCount c = Ledger(dir, day1);
            Check(inside && sent == 1 && r1 && r2 && j1.size() == 300 && j1 == j2 && c.asked == 1 &&
                      c.tiles == 1,
                  Fmt("two threads ask for one tile at once: the request is made %d time(s), the "
                      "second thread waits on the first's fetch (%u wait) and both get the tile "
                      "(%zu and %zu bytes); the ledger reads %llu asked, %llu tile",
                      sent.load(), P.FlightWaits(), j1.size(), j2.size(), U(c.asked), U(c.tiles)));
        }
        // 16. eight threads, four tiles, each tile asked by two.
        {
            const std::string dir = flightCase("flight8");
            Latch latch;
            std::atomic<int> sent{0}, ok{0};
            GoogleTileProvider P;
            P.InitCacheOnly(dir, "satellite", 100, &caps, clock, lands(latch, sent, 0x32));
            std::vector<std::vector<uint8_t>> got(8);
            std::vector<std::thread> ts;
            for (int i = 0; i < 8; ++i) {
                ts.emplace_back([&, i] {
                    if (P.Fetch(13, 2100 + i % 4, 3000, got[i]) && got[i].size() == 300) ok.fetch_add(1);
                });
            }
            const bool inside = Until([&] { return sent.load() == 4 && P.FlightWaits() >= 4; });
            latch.Open();
            for (std::thread& t : ts) t.join();
            const DayCount c = Ledger(dir, day1);
            Check(inside && sent == 4 && ok == 8 && c.asked == 4 && c.tiles == 4,
                  Fmt("eight threads, four tiles, each asked by two: %d requests, %d of 8 threads "
                      "get their tile; the ledger reads %llu asked, %llu tiles",
                      sent.load(), ok.load(), U(c.asked), U(c.tiles)));
        }
        // 17. a request that fails while a second thread waits on it.
        {
            const std::string dir = flightCase("flightfail");
            Latch latch;
            std::atomic<int> sent{0};
            GoogleTileProvider P;
            P.InitCacheOnly(dir, "satellite", 100, &caps, clock, [&](std::vector<uint8_t>&) {
                sent.fetch_add(1);
                latch.Wait();
                return DaySent::Failed;
            });
            std::vector<uint8_t> j1, j2;
            bool r1 = true, r2 = true;
            std::thread t1([&] { r1 = P.Fetch(13, 2200, 3000, j1); });
            Until([&] { return sent.load() == 1; });
            std::thread t2([&] { r2 = P.Fetch(13, 2200, 3000, j2); });
            const bool inside = Until([&] { return P.FlightWaits() >= 1; });
            latch.Open();
            t1.join();
            t2.join();
            const DayCount c = Ledger(dir, day1);
            const uint32_t row = P.FailRow();
            Check(inside && sent == 1 && !r1 && !r2 && c.asked == 1 && c.tiles == 0 && row == 1,
                  Fmt("a request that fails while a second thread waits on it: %d request sent, "
                      "both threads get the failure (%s, %s); the ledger reads %llu asked, %llu "
                      "tiles, and the row of failures is %u",
                      sent.load(), r1 ? "LANDED" : "failed", r2 ? "LANDED" : "failed", U(c.asked),
                      U(c.tiles), row));
        }
        // 18. PLANTED: the rule with the in-flight set taken out, against case 15.
        {
            const std::string dir = flightCase("noflight");
            Latch latch;
            std::atomic<int> sent{0};
            GoogleTileProvider P;
            P.InitCacheOnly(dir, "satellite", 100, &caps, clock, lands(latch, sent, 0x33));
            std::vector<uint8_t> j1, j2;
            std::thread t1([&] { P.Fetch(13, 2300, 3000, j1, false); });
            Until([&] { return sent.load() == 1; });
            std::thread t2([&] { P.Fetch(13, 2300, 3000, j2, false); });
            Until([&] { return sent.load() == 2 || P.FlightWaits() >= 1; });
            latch.Open();
            t1.join();
            t2.join();
            const DayCount c = Ledger(dir, day1);
            Caught(sent == 2 && c.asked == 2,
                   Fmt("PLANTED, the rule with the in-flight set taken out, two threads and one "
                       "tile as in 15: the request is made %d times and the ledger reads %llu "
                       "asked -- the first real run's duplicate",
                       sent.load(), U(c.asked)));
        }
    }

    // ---- the scratch: the files this test made, then its folders; no recursion ----------------
    // Folders in the reverse of their making, so a case's satellite\ goes before the case.
    if (g_ok) {
        bool gone = true;
        size_t files = 0;
        for (auto dit = dirs.rbegin(); dit != dirs.rend(); ++dit) {
            const std::string& d = *dit;
            WIN32_FIND_DATAA fd;
            const HANDLE h = FindFirstFileA((d + "\\*").c_str(), &fd);
            if (h != INVALID_HANDLE_VALUE) {
                do {
                    // "." and ".." and anything that is not a plain file are left alone: the
                    // cases make files only, so a folder here would be a failure to see.
                    if (fd.dwFileAttributes & (FILE_ATTRIBUTE_DIRECTORY | FILE_ATTRIBUTE_REPARSE_POINT)) {
                        continue;
                    }
                    const bool del = DeleteFileA((d + "\\" + fd.cFileName).c_str()) != 0;
                    gone &= del;
                    files += del ? 1 : 0;
                } while (FindNextFileA(h, &fd));
                FindClose(h);
            }
            gone &= RemoveDirectoryA(d.c_str()) != 0;
        }
        gone &= RemoveDirectoryA(run.c_str()) != 0;
        Check(gone, Fmt("the scratch removed: %zu files in %zu folders, one folder at a time",
                        files, dirs.size()));
    }
    if (!g_ok) Log("[daytest]   the scratch is left for inspection: %s", run.c_str());
    Log("[daytest] ---- %s: the count meets its cap and the next fetch is refused; the bytes' cap "
        "can close the day first; zero is no request; the day turns at a fetch; two writers lose "
        "nothing; a damaged ledger is a met cap; a failure is asked, not a tile, and one never "
        "sent adds nothing; the provider stops asking after %u failures in a row and asks its "
        "budget before the ledger; one thread at a time fetches a tile and the others take its "
        "result; a request's log form holds no part of its query; the four planted rules were "
        "caught ----",
        g_ok ? "PASS" : "FAIL", GoogleTileProvider::kFailRow);
    return g_ok;
}

}  // namespace ga
