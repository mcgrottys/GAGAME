#include "core/DayLedger.h"

#include "core/Common.h"
#include "core/Json.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

#include <cmath>
#include <cstdio>
#include <ctime>
#include <fstream>

namespace ga {

namespace {

std::string UtcText(long long t, bool withTime) {
    const time_t tt = static_cast<time_t>(t);
    std::tm tm{};
    if (gmtime_s(&tm, &tt) != 0) tm = std::tm{0, 0, 0, 1, 0, 70};
    char b[32];
    if (withTime) {
        snprintf(b, sizeof b, "%04d-%02d-%02dT%02d:%02d:%02dZ", tm.tm_year + 1900, tm.tm_mon + 1,
                 tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);
    } else {
        snprintf(b, sizeof b, "%04d-%02d-%02d", tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday);
    }
    return b;
}

bool CountOf(const JsonValue* n, uint64_t& out) {
    if (!n || n->type != JsonValue::Type::Number) return false;
    const double v = n->number;
    if (!(v >= 0.0) || v >= 1.8e19 || v != std::floor(v)) return false;
    out = static_cast<uint64_t>(v);
    return true;
}

unsigned long long U(uint64_t v) { return static_cast<unsigned long long>(v); }
double Gb(uint64_t v) { return static_cast<double>(v) / 1e9; }

}  // namespace

DayLedger::~DayLedger() {
    if (m_lock) CloseHandle(static_cast<HANDLE>(m_lock));
}

bool DayLedger::Open(const std::string& folder, const DayCaps& caps, const char* tag, Clock clock) {
    m_folder = folder;
    m_caps = caps;
    m_tag = tag ? tag : "[day]";
    m_clock = clock ? std::move(clock) : Clock([] { return static_cast<long long>(time(nullptr)); });
    m_open = true;
    m_lock = CreateMutexW(nullptr, FALSE, kLock);
    if (!m_lock) {
        m_broken = true;
        Log("%s the day ledger's lock %S cannot be made (error %lu): EVERY request is refused this "
            "run",
            m_tag.c_str(), kLock, GetLastError());
        return false;
    }
    // THE BOOT LINE: what today's ledger holds, and the caps. No URL, no key.
    const std::string day = Today(), path = PathOf(day);
    DayCount c;
    std::string why;
    const DayRead r = Read(path, c, &why);
    if (r == DayRead::Damaged) {
        Log("%s today's ledger %s is DAMAGED (%s): the day's cap is treated as MET; the caps are "
            "%llu requests and %llu bytes (%.3f GB) a UTC day",
            m_tag.c_str(), path.c_str(), why.c_str(), U(m_caps.tiles), U(m_caps.bytes),
            Gb(m_caps.bytes));
    } else {
        const bool met = c.asked >= m_caps.tiles || c.bytes >= m_caps.bytes;
        Log("%s today (UTC %s) the ledger %s holds %llu asked, %llu tiles, %llu bytes (%.3f GB)%s; "
            "the caps are %llu requests and %llu bytes (%.3f GB) a day, every engine on the "
            "machine together%s",
            m_tag.c_str(), day.c_str(), path.c_str(), U(c.asked), U(c.tiles), U(c.bytes),
            Gb(c.bytes), r == DayRead::Missing ? " (no file yet)" : "", U(m_caps.tiles),
            U(m_caps.bytes), Gb(m_caps.bytes),
            met ? " -- a cap is MET: no request until the UTC day turns" : "");
    }
    return true;
}

std::string DayLedger::Today() const {
    return UtcText(m_clock ? m_clock() : static_cast<long long>(time(nullptr)), false);
}

std::string DayLedger::PathOf(const std::string& day) const {
    return m_folder + "\\day_" + day + ".json";
}

DayRead DayLedger::Read(const std::string& path, DayCount& c, std::string* why) {
    c = DayCount{};
    auto damaged = [&](const std::string& w) {
        c = DayCount{};
        if (why) *why = w;
        return DayRead::Damaged;
    };
    HANDLE h = INVALID_HANDLE_VALUE;
    DWORD err = 0;
    for (int attempt = 0; attempt < 20; ++attempt) {
        h = CreateFileA(path.c_str(), GENERIC_READ,
                        FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                        OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
        if (h != INVALID_HANDLE_VALUE) break;
        err = GetLastError();
        if (err == ERROR_FILE_NOT_FOUND || err == ERROR_PATH_NOT_FOUND) return DayRead::Missing;
        if (err != ERROR_SHARING_VIOLATION && err != ERROR_ACCESS_DENIED) break;
        Sleep(5);   // held a moment by a scanner or a reader that does not share
    }
    if (h == INVALID_HANDLE_VALUE) return damaged("cannot be opened, error " + std::to_string(err));
    char buf[1024];
    DWORD got = 0;
    const BOOL ok = ReadFile(h, buf, sizeof buf, &got, nullptr);
    CloseHandle(h);
    if (!ok) return damaged("cannot be read");
    if (got == sizeof buf) return damaged("longer than a ledger is");
    std::string err2;
    const JsonValue v = JsonParser::Parse(std::string(buf, got), &err2);
    if (!err2.empty()) return damaged("not whole: " + err2 + " in " + std::to_string(got) + " bytes");
    if (!CountOf(v.Get("asked"), c.asked) || !CountOf(v.Get("tiles"), c.tiles) ||
        !CountOf(v.Get("bytes"), c.bytes)) {
        return damaged("no whole asked, tiles and bytes counts");
    }
    if (c.asked < c.tiles) return damaged("fewer asked than tiles");
    c.last = v.Str("last");
    return DayRead::Whole;
}

bool DayLedger::Acquire() {
    const HANDLE h = static_cast<HANDLE>(m_lock);
    if (!h) return false;
    DWORD w = WaitForSingleObject(h, 0);
    if (w == WAIT_TIMEOUT) {
        m_waits.fetch_add(1, std::memory_order_relaxed);
        w = WaitForSingleObject(h, kLockWaitMs);
    }
    // WAIT_ABANDONED: the holder died holding it. The lock is ours, and the ledger is whole:
    // it is only ever replaced by a rename.
    if (w == WAIT_OBJECT_0 || w == WAIT_ABANDONED) return true;
    if (!m_saidLock.exchange(true)) {
        Log("%s the day ledger's lock was not had in %lu ms (wait %lu): that request is refused",
            m_tag.c_str(), kLockWaitMs, w);
    }
    return false;
}

void DayLedger::Release() { ReleaseMutex(static_cast<HANDLE>(m_lock)); }

DayFetch DayLedger::Fetch(const Request& request, std::vector<uint8_t>& body) {
    body.clear();
    if (!m_open) {
        const DaySent s = request(body);
        return s == DaySent::Landed   ? DayFetch::Fetched
               : s == DaySent::Failed ? DayFetch::Failed
                                      : DayFetch::NotSent;
    }
    if (!Allow()) return DayFetch::Refused;
    const DaySent s = request(body);
    if (s == DaySent::No) return DayFetch::NotSent;   // never sent: adds nothing
    // Sent: asked + 1, and a tile and its bytes if it landed whole. A write that fails is logged
    // and refuses the run's later requests.
    Add(s == DaySent::Landed, body.size());
    return s == DaySent::Landed ? DayFetch::Fetched : DayFetch::Failed;
}

bool DayLedger::Allow(DayCount* seen) {
    if (!m_open) return true;
    if (m_broken || !Acquire()) return false;
    struct Held { DayLedger* L; ~Held() { L->Release(); } } held{this};
    m_reads.fetch_add(1, std::memory_order_relaxed);
    const std::string day = Today(), path = PathOf(day);
    DayCount c;
    std::string why;
    const DayRead r = Read(path, c, &why);
    if (seen) *seen = c;
    if (r == DayRead::Damaged) {
        if (m_saidDamaged != day) {
            m_saidDamaged = day;
            Log("%s today's ledger %s is DAMAGED (%s): the day's cap is treated as MET -- no "
                "request until the file is whole or the UTC day turns",
                m_tag.c_str(), path.c_str(), why.c_str());
        }
        return false;
    }
    if (c.asked < m_caps.tiles && c.bytes < m_caps.bytes) return true;
    if (m_saidMet != day) {
        m_saidMet = day;
        Log("%s DAY CAP MET (UTC %s): the ledger %s holds %llu asked, %llu tiles, %llu bytes "
            "(%.3f GB); the caps are %llu requests and %llu bytes (%.3f GB) -- no request until the "
            "UTC day turns; the cache still serves",
            m_tag.c_str(), day.c_str(), path.c_str(), U(c.asked), U(c.tiles), U(c.bytes),
            Gb(c.bytes), U(m_caps.tiles), U(m_caps.bytes), Gb(m_caps.bytes));
    }
    return false;
}

bool DayLedger::Add(bool landed, uint64_t bytes) {
    if (!m_open) return true;
    if (!Acquire()) {
        // Fail closed: a request this engine cannot count must be its last.
        m_broken = true;
        Log("%s a request is NOT counted (the lock was not had); every later request of the run "
            "is refused",
            m_tag.c_str());
        return false;
    }
    struct Held { DayLedger* L; ~Held() { L->Release(); } } held{this};
    m_reads.fetch_add(1, std::memory_order_relaxed);
    // THE RE-READ: the ledger as it is NOW, not as it was before the request -- another engine
    // may have written it while this one waited on the network.
    const long long now = m_clock();
    const std::string day = UtcText(now, false), path = PathOf(day);
    DayCount c;
    std::string why;
    if (Read(path, c, &why) == DayRead::Damaged) {
        if (m_saidDamaged != day) {
            m_saidDamaged = day;
            Log("%s today's ledger %s is DAMAGED (%s): a request in flight is not counted, and "
                "the day's cap is treated as MET",
                m_tag.c_str(), path.c_str(), why.c_str());
        }
        return false;
    }
    c.asked += 1;
    if (landed) {
        c.tiles += 1;
        c.bytes += bytes;
    }
    c.last = UtcText(now, true);
    return WriteLocked(day, c);
}

bool DayLedger::Put(const std::string& day, const DayCount& c) {
    if (!Acquire()) return false;
    struct Held { DayLedger* L; ~Held() { L->Release(); } } held{this};
    return WriteLocked(day, c);
}

bool DayLedger::WriteLocked(const std::string& day, const DayCount& c) {
    const std::string path = PathOf(day);
    const std::string tmp = path + "." + std::to_string(GetCurrentProcessId()) + ".tmp";
    char text[320];
    const int n = snprintf(text, sizeof text,
                           "{\"day\": \"%s\", \"asked\": %llu, \"tiles\": %llu, \"bytes\": %llu, "
                           "\"last\": \"%s\"}\n",
                           day.c_str(), U(c.asked), U(c.tiles), U(c.bytes), c.last.c_str());
    bool wrote = n > 0 && n < static_cast<int>(sizeof text);
    if (wrote) {
        std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
        f.write(text, n);
        f.close();
        wrote = !f.fail();
    }
    DWORD err = 0;
    for (int attempt = 0; wrote && attempt < 20; ++attempt) {
        // The rename is the whole write: a reader sees the old file or the new one, never half.
        if (MoveFileExA(tmp.c_str(), path.c_str(),
                        MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH)) {
            return true;
        }
        err = GetLastError();
        Sleep(5);   // the old file held a moment by a reader that does not share delete
    }
    DeleteFileA(tmp.c_str());   // this engine's own temporary file, one file
    m_broken = true;
    Log("%s the day ledger %s could not be written (error %lu): a request is NOT counted, and "
        "every later request of the run is refused",
        m_tag.c_str(), path.c_str(), err);
    return false;
}

}  // namespace ga
