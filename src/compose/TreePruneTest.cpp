// ================================================================================================
//  TreePruneTest - the [prunetest] block of --selftest: the prune tool's refusals, each planted and
//  caught, on a scratch root this test makes under the worktree's own out\ -- never in the cache.
//
//  THE SCRATCH is out\prunetest\<UTC stamp>_<pid>\: trees\ (the root the tool is pointed at), a
//  canary folder with one file OUTSIDE that root, junctions into the canary from a node's place, a
//  tag's place, the inside of a stale tag folder and the inside of an old retired batch, a junction
//  named cache for the confirm case and a folder named google for the root case. Every junction
//  points INSIDE the run folder, so nothing anyone later does to out\ can be led out of it. When
//  every check has passed, the test unlinks its own junctions and the tool's own walk removes the
//  run folder; when one has failed, the folder is left and named, for inspection.
//
//  THE CLOCK is the real one, with the file times set around it by SetFileTime: tag folders whose
//  files are 10, 40 and 70 days old, stamps of today and of 40 days ago, batches retired 3, 10 and
//  12 days ago. THE PROCESS TABLE is injected: one naming only this engine, so another session's
//  render on this machine cannot change a verdict, and planted ones with a second gagame.exe or
//  none that can be read. The real table is asked once, to see that it can see an engine at all.
// ================================================================================================
#include "compose/TreePrune.h"

#ifndef NOMINMAX
#define NOMINMAX   // windows.h's min/max macros would break std::min in every includer
#endif
#include <windows.h>
#include <winioctl.h>
#undef min
#undef max

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "core/Common.h"
#include "core/Json.h"

namespace ga {

namespace {

using namespace prune;

constexpr uint64_t kDay = 864000000000ull;   // FILETIME ticks in a day
constexpr uint64_t kSecond = 10000000ull;

bool g_ok = true;
void Check(bool cond, const std::string& what) {
    if (!cond) {
        Log("[prunetest]   FAIL: %s", what.c_str());
        g_ok = false;
    }
}
// A planted failure: the tool must refuse, and the line says what it refused.
void Caught(bool cond, const std::string& what) {
    if (cond) {
        Log("[prunetest] CAUGHT: %s", what.c_str());
    } else {
        Log("[prunetest]   FAIL (MISSED): %s", what.c_str());
        g_ok = false;
    }
}

std::string Fmt(const char* fmt, ...) {
    char b[512];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    return b;
}
FILETIME Ft(uint64_t t) {
    FILETIME f;
    f.dwLowDateTime = static_cast<DWORD>(t & 0xFFFFFFFFull);
    f.dwHighDateTime = static_cast<DWORD>(t >> 32);
    return f;
}
uint64_t Ago(uint64_t now, double days) { return now - static_cast<uint64_t>(days * double(kDay)); }
std::string Iso(uint64_t t) {
    const FILETIME f = Ft(t);
    SYSTEMTIME s{};
    FileTimeToSystemTime(&f, &s);
    char b[32];
    snprintf(b, sizeof(b), "%04u-%02u-%02uT%02u:%02u:%02uZ", s.wYear, s.wMonth, s.wDay, s.wHour,
             s.wMinute, s.wSecond);
    return b;
}
bool Exists(const std::wstring& p) { return GetFileAttributesW(p.c_str()) != INVALID_FILE_ATTRIBUTES; }

bool MakeDir(const std::wstring& p) {
    return CreateDirectoryW(p.c_str(), nullptr) || GetLastError() == ERROR_ALREADY_EXISTS;
}
// A file of these bytes, created and last written `ageDays` before `now`. The time is set on the
// writing handle before it closes, which is what makes it stick.
bool MakeFile(const std::wstring& p, const std::string& bytes, uint64_t now, double ageDays) {
    const HANDLE h = CreateFileW(p.c_str(), GENERIC_WRITE, 0, nullptr, CREATE_ALWAYS,
                                 FILE_ATTRIBUTE_NORMAL, nullptr);
    if (h == INVALID_HANDLE_VALUE) return false;
    DWORD wrote = 0;
    const bool ok = WriteFile(h, bytes.data(), static_cast<DWORD>(bytes.size()), &wrote, nullptr) &&
                    wrote == bytes.size();
    const FILETIME f = Ft(Ago(now, ageDays));
    const bool timed = SetFileTime(h, &f, nullptr, &f) != 0;
    CloseHandle(h);
    return ok && timed;
}
// A junction (a mount-point reparse point) at `link` to the folder `target`: no privilege is
// needed, unlike a symbolic link. REPARSE_DATA_BUFFER lives in the driver kit's headers, so its
// mount-point arm is laid out here by hand: tag, data length, reserved, then the substitute name
// (an NT path) and the print name, each NUL-terminated.
bool MakeJunction(const std::wstring& link, const std::wstring& target) {
    if (!CreateDirectoryW(link.c_str(), nullptr)) return false;
    const std::wstring sub = L"\\??\\" + target;
    const size_t subB = sub.size() * sizeof(wchar_t), printB = target.size() * sizeof(wchar_t);
    std::vector<uint8_t> buf(16 + subB + 2 + printB + 2, 0);
    auto put16 = [&buf](size_t at, size_t v) {
        buf[at] = static_cast<uint8_t>(v & 0xFF);
        buf[at + 1] = static_cast<uint8_t>((v >> 8) & 0xFF);
    };
    const DWORD tag = IO_REPARSE_TAG_MOUNT_POINT;
    memcpy(buf.data(), &tag, 4);
    put16(4, buf.size() - 8);   // ReparseDataLength: everything after the 8-byte header
    put16(8, 0);                // SubstituteNameOffset
    put16(10, subB);            // SubstituteNameLength
    put16(12, subB + 2);        // PrintNameOffset
    put16(14, printB);          // PrintNameLength
    memcpy(buf.data() + 16, sub.c_str(), subB);
    memcpy(buf.data() + 16 + subB + 2, target.c_str(), printB);
    const HANDLE h = CreateFileW(link.c_str(), GENERIC_WRITE, 0, nullptr, OPEN_EXISTING,
                                 FILE_FLAG_OPEN_REPARSE_POINT | FILE_FLAG_BACKUP_SEMANTICS, nullptr);
    bool ok = false;
    if (h != INVALID_HANDLE_VALUE) {
        DWORD got = 0;
        ok = DeviceIoControl(h, FSCTL_SET_REPARSE_POINT, buf.data(), static_cast<DWORD>(buf.size()),
                             nullptr, 0, &got, nullptr) != 0;
        CloseHandle(h);
    }
    if (!ok) RemoveDirectoryW(link.c_str());   // still the empty folder made just above
    return ok;
}

// ---- the snapshot: what "nothing changed" is measured with ------------------------------------
struct Row {
    std::wstring rel;
    DWORD attrs = 0;
    uint64_t size = 0, created = 0, written = 0, hash = 0;
};
uint64_t HashFile(const std::wstring& p) {
    const HANDLE h = CreateFileW(p.c_str(), GENERIC_READ,
                                 FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                                 OPEN_EXISTING, FILE_FLAG_OPEN_REPARSE_POINT, nullptr);
    if (h == INVALID_HANDLE_VALUE) return 0;
    uint64_t x = 14695981039346656037ull;
    uint8_t buf[4096];
    DWORD got = 0;
    while (ReadFile(h, buf, sizeof(buf), &got, nullptr) && got) {
        for (DWORD i = 0; i < got; ++i) {
            x ^= buf[i];
            x *= 1099511628211ull;
        }
    }
    CloseHandle(h);
    return x;
}
// Every entry under `dir`: attributes, size, creation and last write, and a file's bytes' hash
// (last ACCESS is left out: it is the one time a read may move, and nothing here is judged by
// it). A reparse point is recorded and not entered, so the snapshot never reaches through one.
//
// THE NAMES COME FROM THE LISTING AND THE TIMES FROM EACH ENTRY ITSELF. A directory listing
// carries a copy of each child's times in the parent's index, and NTFS refreshes that copy
// lazily -- when a handle on the child closes -- so a folder merely LISTED by the tool can show a
// "new" write time in the parent's listing without anything having been written. Measured: the
// first version of this snapshot, reading the listing's times, called the listing a change to
// .retired. GetFileAttributesExW reads the entry's own record (and, on a link, the link's).
void Snap(const std::wstring& dir, const std::wstring& rel, std::vector<Row>& out, int depth) {
    if (depth > 16) return;
    WIN32_FIND_DATAW fd{};
    const HANDLE h = FindFirstFileExW((dir + L"\\*").c_str(), FindExInfoBasic, &fd,
                                      FindExSearchNameMatch, nullptr, 0);
    if (h == INVALID_HANDLE_VALUE) return;
    std::vector<std::wstring> names;
    do {
        if (wcscmp(fd.cFileName, L".") == 0 || wcscmp(fd.cFileName, L"..") == 0) continue;
        names.push_back(fd.cFileName);
    } while (FindNextFileW(h, &fd));
    FindClose(h);
    for (const std::wstring& name : names) {
        const std::wstring path = dir + L"\\" + name;
        WIN32_FILE_ATTRIBUTE_DATA a{};
        if (!GetFileAttributesExW(path.c_str(), GetFileExInfoStandard, &a)) continue;
        Row r;
        r.rel = rel + name;
        r.attrs = a.dwFileAttributes;
        r.size = (uint64_t(a.nFileSizeHigh) << 32) | a.nFileSizeLow;
        r.created = (uint64_t(a.ftCreationTime.dwHighDateTime) << 32) | a.ftCreationTime.dwLowDateTime;
        r.written = (uint64_t(a.ftLastWriteTime.dwHighDateTime) << 32) | a.ftLastWriteTime.dwLowDateTime;
        const bool link = (r.attrs & FILE_ATTRIBUTE_REPARSE_POINT) != 0;
        const bool folder = (r.attrs & FILE_ATTRIBUTE_DIRECTORY) != 0;
        if (!link && !folder) r.hash = HashFile(path);
        out.push_back(r);
        if (folder && !link) Snap(path, r.rel + L"\\", out, depth + 1);
    }
}
std::vector<Row> SnapAll(const std::wstring& dir) {
    std::vector<Row> out;
    Snap(dir, L"", out, 0);
    std::sort(out.begin(), out.end(), [](const Row& a, const Row& b) { return a.rel < b.rel; });
    return out;
}
// Equal row for row. `folderWrites` false leaves a folder's last write out: moving an entry out
// of a folder is a write to that folder, and says nothing about the entries still in it.
bool Same(const std::vector<Row>& a, const std::vector<Row>& b, bool folderWrites, std::string* first) {
    const size_t n = (std::max)(a.size(), b.size());
    for (size_t i = 0; i < n; ++i) {
        if (i >= a.size() || i >= b.size()) {
            if (first) *first = Narrow((i < a.size() ? a[i] : b[i]).rel) + (i < a.size() ? " is gone" : " appeared");
            return false;
        }
        const Row& x = a[i];
        const Row& y = b[i];
        const bool folder = (x.attrs & FILE_ATTRIBUTE_DIRECTORY) != 0;
        const bool same = x.rel == y.rel && x.attrs == y.attrs && x.size == y.size &&
                          x.created == y.created && x.hash == y.hash &&
                          (x.written == y.written || (folder && !folderWrites));
        if (!same) {
            if (first) *first = Narrow(x.rel) + " vs " + Narrow(y.rel);
            return false;
        }
    }
    return true;
}
bool Starts(const std::wstring& s, const std::wstring& p) { return s.compare(0, p.size(), p) == 0; }
bool Ends(const std::wstring& s, const std::wstring& p) {
    return s.size() >= p.size() && s.compare(s.size() - p.size(), p.size(), p) == 0;
}

const Entry* Find(const Result& r, const char* node, const char* tag) {
    for (const Entry& e : r.listing.entries) {
        if (e.node == node && e.tag == tag) return &e;
    }
    return nullptr;
}
const Skipped* Skip(const std::vector<Skipped>& v, const std::wstring& path) {
    for (const Skipped& s : v) {
        if (s.path == path) return &s;
    }
    return nullptr;
}
bool Has(const std::string& s, const char* part) { return s.find(part) != std::string::npos; }

}  // namespace

bool RunPruneSelfTest() {
    g_ok = true;
    Log("[prunetest] ---- the tree-prune tool on a scratch root under out\\prunetest, a canary "
        "behind junctions ----");

    // ---- where: the worktree's own out\, a plain folder -- or nothing is written at all ----------
    std::vector<wchar_t> cwd(32768, L'\0');
    const DWORD cwdLen = GetCurrentDirectoryW(static_cast<DWORD>(cwd.size()), cwd.data());
    if (cwdLen == 0 || cwdLen >= cwd.size()) {
        Log("[prunetest]   FAIL: the working folder cannot be read, so the test writes nothing");
        return false;
    }
    const std::wstring outDir = std::wstring(cwd.data(), cwdLen) + L"\\out";
    MakeDir(outDir);
    const DWORD oa = GetFileAttributesW(outDir.c_str());
    std::wstring out;
    std::string why;
    if (oa == INVALID_FILE_ATTRIBUTES || !(oa & FILE_ATTRIBUTE_DIRECTORY) ||
        (oa & FILE_ATTRIBUTE_REPARSE_POINT) || !FinalPath(outDir, out, &why)) {
        Log("[prunetest]   FAIL: out\\ is not a plain folder here, so the test writes nothing");
        return false;
    }
    const std::wstring base = out + L"\\prunetest";
    MakeDir(base);
    const DWORD ba = GetFileAttributesW(base.c_str());
    if (ba == INVALID_FILE_ATTRIBUTES || !(ba & FILE_ATTRIBUTE_DIRECTORY) ||
        (ba & FILE_ATTRIBUTE_REPARSE_POINT)) {
        Log("[prunetest]   FAIL: out\\prunetest is not a plain folder, so the test writes nothing");
        return false;
    }
    const uint64_t now = Now();
    const std::wstring run = base + L"\\" + Widen(BatchName(now)) + L"_" + std::to_wstring(GetCurrentProcessId());
    if (!CreateDirectoryW(run.c_str(), nullptr)) {
        Log("[prunetest]   FAIL: cannot make a new scratch folder %s", Narrow(run).c_str());
        return false;
    }
    const std::wstring root = run + L"\\trees", canary = run + L"\\canary";
    std::vector<std::wstring> links;   // every junction this test made, unlinked at the end

    // ---- the scratch --------------------------------------------------------------------------
    bool built = true;
    const std::string tile(64, 't');
    auto dir = [&](const std::wstring& p) { built &= MakeDir(p); };
    auto file = [&](const std::wstring& p, double age, const std::string& bytes) {
        built &= MakeFile(p, bytes, now, age);
    };
    auto stampText = [&](double age) {
        return "{\"utc\": \"" + Iso(Ago(now, age)) + "\", \"rev\": \"prunetest\", \"scene\": "
               "\"prunetest\", \"file\": \"\"}\n";
    };
    auto stamp = [&](const std::wstring& tagDir, double age) {
        file(tagDir + L"\\.live", age, stampText(age));
    };
    auto junction = [&](const std::wstring& link, const std::wstring& target) {
        const bool ok = MakeJunction(link, target);
        built &= ok;
        if (ok) links.push_back(link);
    };
    dir(canary);
    file(canary + L"\\canary.bin", 70, "the canary: nothing the tool does may reach this file");
    dir(root);
    // alpha: a stamp of today on files 70 days old (a painted tree read every day and written
    // never), a stamp of 40 days ago, and a tag's place held by a junction. Both stamped folders
    // carry both archives: <tag>.gaa, and <tag>.gaa.stale, the kind set aside by hand on
    // 2026-09-28 because it held tiles its loose files no longer hold.
    const std::wstring alpha = root + L"\\alpha.0000000a";
    dir(alpha);
    dir(alpha + L"\\cube16k");
    file(alpha + L"\\cube16k\\f0_m0_x0_y0.bin", 70, tile);
    file(alpha + L"\\cube16k\\f0_m1_x0_y0.void", 70, "");
    stamp(alpha + L"\\cube16k", 0);
    file(alpha + L"\\cube16k.gaa", 70, tile);
    file(alpha + L"\\cube16k.gaa.stale", 70, tile);
    dir(alpha + L"\\window_z14_1_2");
    file(alpha + L"\\window_z14_1_2\\f6_m0_x1_y2_0badc0de.bin", 40, tile);
    stamp(alpha + L"\\window_z14_1_2", 40);
    file(alpha + L"\\window_z14_1_2.gaa", 40, tile);
    file(alpha + L"\\window_z14_1_2.gaa.stale", 40, tile);
    junction(alpha + L"\\window_z16_9_9", canary);
    // beta: unstamped, its newest files 10, 40 and 70 days old; the stale one carries only the
    // archive set aside. Then a stale folder beside a name that is neither of its archives, and
    // archives with no folder at all.
    const std::wstring beta = root + L"\\beta.0000000b";
    dir(beta);
    dir(beta + L"\\cube16k");
    file(beta + L"\\cube16k\\f0_m0_x0_y0.bin", 10, tile);
    file(beta + L"\\cube16k.gaa", 10, tile);
    dir(beta + L"\\window_z14_1_2");
    file(beta + L"\\window_z14_1_2\\f6_m0_x0_y0.bin", 40, tile);
    file(beta + L"\\window_z14_1_2\\f6_m1_x0_y0.fold", 40, "");
    dir(beta + L"\\window_z17_3_4");
    file(beta + L"\\window_z17_3_4\\f7_m0_x3_y4.bin", 70, tile);
    file(beta + L"\\window_z17_3_4\\f7_m0_x3_y5_00000001.ref-0000000a", 70, "");
    file(beta + L"\\window_z17_3_4.gaa.stale", 70, tile);
    dir(beta + L"\\window_z16_5_6");
    file(beta + L"\\window_z16_5_6\\f6_m0_x5_y6.bin", 70, tile);
    file(beta + L"\\window_z16_5_6.gaa.bak", 70, tile);
    file(beta + L"\\orphan.gaa", 70, tile);
    file(beta + L"\\gone_z14_0_0.gaa.stale", 70, tile);
    // gamma: 70 days stale, but one tag folder holds THE junction to the canary and one a folder.
    const std::wstring gamma = root + L"\\gamma.0000000c";
    dir(gamma);
    dir(gamma + L"\\cube16k");
    file(gamma + L"\\cube16k\\f0_m0_x0_y0.bin", 70, tile);
    junction(gamma + L"\\cube16k\\link", canary);
    dir(gamma + L"\\window_z14_1_2");
    file(gamma + L"\\window_z14_1_2\\f6_m0_x0_y0.bin", 70, tile);
    dir(gamma + L"\\window_z14_1_2\\nested");
    file(gamma + L"\\window_z14_1_2\\nested\\f6_m0_x1_y0.bin", 70, tile);
    // A node's place held by a junction, names that are not a node's, and a stray note in the
    // root named as the one the archives' setting-aside left in the real cache.
    junction(root + L"\\delta.0000000d", canary);
    dir(root + L"\\Not-A-Node");
    dir(root + L"\\Not-A-Node\\cube16k");
    file(root + L"\\Not-A-Node\\cube16k\\f0_m0_x0_y0.bin", 70, tile);
    dir(root + L"\\UPPER.0000000A");
    dir(root + L"\\UPPER.0000000A\\cube16k");
    file(root + L"\\UPPER.0000000A\\cube16k\\f0_m0_x0_y0.bin", 70, tile);
    const std::wstring note = root + L"\\ARCHIVES_SET_ASIDE_2026-09-28.txt";
    file(note, 70, "a note left in the root by hand\r\n");
    // .retired: batches retired 3, 10 and 12 days ago -- the last holding a junction two levels
    // down -- one named like a batch of 20 days ago but with no manifest (not retire's), and a
    // folder that is not a batch.
    const std::wstring retired = root + L"\\.retired";
    const std::wstring b3 = retired + L"\\" + Widen(BatchName(Ago(now, 3))),
                       b10 = retired + L"\\" + Widen(BatchName(Ago(now, 10))),
                       b12 = retired + L"\\" + Widen(BatchName(Ago(now, 12))),
                       b20 = retired + L"\\" + Widen(BatchName(Ago(now, 20)));
    dir(retired);
    dir(b3);
    dir(b3 + L"\\alpha.0000000a");
    dir(b3 + L"\\alpha.0000000a\\cube16k");
    file(b3 + L"\\alpha.0000000a\\cube16k\\f0_m0_x0_y0.bin", 50, tile);
    file(b3 + L"\\manifest.json", 3, "{}\n");
    dir(b10);
    dir(b10 + L"\\beta.0000000b");
    dir(b10 + L"\\beta.0000000b\\cube16k");
    file(b10 + L"\\beta.0000000b\\cube16k\\f0_m0_x0_y0.bin", 80, tile);
    file(b10 + L"\\manifest.json", 10, "{}\n");
    dir(b12);
    dir(b12 + L"\\gamma.0000000c");
    dir(b12 + L"\\gamma.0000000c\\cube16k");
    file(b12 + L"\\gamma.0000000c\\cube16k\\f0_m0_x0_y0.bin", 80, tile);
    file(b12 + L"\\manifest.json", 12, "{}\n");
    junction(b12 + L"\\gamma.0000000c\\cube16k\\link", canary);
    dir(b20);
    file(b20 + L"\\something-put-here-by-hand.bin", 20, tile);
    dir(retired + L"\\not-a-batch");
    file(retired + L"\\not-a-batch\\f0_m0_x0_y0.bin", 80, tile);
    if (!built) {
        Log("[prunetest]   FAIL: the scratch could not be built in %s (left for inspection)",
            Narrow(run).c_str());
        return false;
    }
    const std::vector<Row> canary0 = SnapAll(canary);
    std::string first;

    // The tool's own lines, kept; `echo` also prints them under the test's prefix.
    std::vector<std::string> said;
    bool echo = false;
    const auto capture = [&said, &echo](const std::string& s) {
        said.push_back(s);
        if (echo) Log("[prunetest]   | %s", s.c_str());
    };
    // Only this engine in the table: another session's render cannot decide a verdict here.
    const auto alone = [](std::vector<Process>& rows) {
        rows.push_back({GetCurrentProcessId(), "gagame.exe"});
        return true;
    };

    // ---- 1. list: every verdict as planted, and nothing changed ------------------------------
    Request q;
    q.root = Narrow(root);
    q.now = now;
    q.processes = alone;
    q.say = capture;
    const std::vector<Row> before = SnapAll(root);
    echo = true;
    const Result L = Run(q);
    echo = false;
    Check(L.status == Status::Listed, "list ends as listed");
    Check(L.listing.root == root, "the listing's root is the scratch root's final path");
    struct Want {
        const char* node;
        const char* tag;
        Verdict v;
    };
    const Want want[] = {{"alpha", "cube16k", Verdict::Keep},
                         {"alpha", "window_z14_1_2", Verdict::Stale},
                         {"beta", "cube16k", Verdict::KeepUnstamped},
                         {"beta", "window_z14_1_2", Verdict::KeepUnstamped},
                         {"beta", "window_z17_3_4", Verdict::StaleUnstamped}};
    int right = 0;
    for (const Want& w : want) {
        const Entry* e = Find(L, w.node, w.tag);
        const bool ok = e && e->verdict == w.v;
        right += ok ? 1 : 0;
        Check(ok, std::string("list: ") + w.node + "\\" + w.tag + " is " + VerdictName(w.v));
    }
    Check(L.listing.entries.size() == 5, "list: exactly the five plain tag folders are trees");
    if (const Entry* e = Find(L, "alpha", "cube16k")) {
        // 64 (the .bin) + 0 (the .void) + the stamp + 64 (.gaa) + 64 (.gaa.stale)
        Check(e->tiles == 2 && e->files == 3 && e->stamped && e->stampRead && e->stampRev == "prunetest" &&
                  !e->gaa.empty() && !e->stale.empty() && e->bytes == 64 + stampText(0).size() + 128,
              "list: alpha\\cube16k counts its two tiles, its stamp and both of its archives");
    }
    if (const Entry* e = Find(L, "beta", "window_z17_3_4")) {
        Check(e->gaa.empty() && !e->stale.empty() && e->bytes == 64 + 0 + 64,
              "list: beta\\window_z17_3_4 counts the archive set aside, and has no other");
    }
    struct Skip1 {
        std::wstring path;
        const char* why;
    };
    const Skip1 skips[] = {{gamma + L"\\cube16k", "reparse point"},
                           {gamma + L"\\window_z14_1_2", "holds a folder"},
                           {root + L"\\delta.0000000d", "reparse point"},
                           {alpha + L"\\window_z16_9_9", "reparse point"},
                           {root + L"\\Not-A-Node", "not named"},
                           {root + L"\\UPPER.0000000A", "not named"},
                           {note, "a file in the root"},
                           {beta + L"\\orphan.gaa", "no tag folder"},
                           {beta + L"\\gone_z14_0_0.gaa.stale", "no tag folder"},
                           {beta + L"\\window_z16_5_6", "neither of its archives"},
                           {beta + L"\\window_z16_5_6.gaa.bak", "a file beside the tag folders"},
                           {retired + L"\\not-a-batch", "not a batch"}};
    int skippedRight = 0;
    for (const Skip1& s : skips) {
        const Skipped* k = Skip(L.listing.skipped, s.path);
        const bool ok = k && Has(k->why, s.why);
        skippedRight += ok ? 1 : 0;
        Check(ok, "list: " + Narrow(s.path) + " SKIPPED (" + s.why + ")");
    }
    Check(L.listing.skipped.size() == sizeof(skips) / sizeof(skips[0]), "list: nothing else SKIPPED");
    bool retiredAsTree = false;
    for (const Entry& e : L.listing.entries) retiredAsTree |= Has(e.node, "retired");
    int made = 0;
    for (const Batch& b : L.listing.batches) made += b.manifest ? 1 : 0;
    Check(!retiredAsTree && L.listing.batches.size() == 4 && made == 3,
          ".retired is never a tree; its four stamp-named folders are batches, three with a manifest");
    const std::vector<Row> after = SnapAll(root);
    const bool listQuiet = Same(before, after, true, &first) && Same(canary0, SnapAll(canary), true, &first);
    Check(listQuiet, "list changed nothing: " + first);
    Log("[prunetest] list at 30 days: %d of 5 verdicts as planted, %d of %zu SKIPPED with the "
        "planted reason (the note in the root among them), 4 batches; %zu entries before and after, "
        "names, sizes, times and bytes %s",
        right, skippedRight, sizeof(skips) / sizeof(skips[0]), before.size(),
        listQuiet ? "equal" : "DIFFERENT");
    {
        Request q14 = q;
        q14.ageDays = 14.0;
        said.clear();
        const Result L14 = Run(q14);
        const Entry* w = Find(L14, "beta", "window_z14_1_2");
        const Entry* c = Find(L14, "beta", "cube16k");
        Check(w && w->verdict == Verdict::StaleUnstamped && c && c->verdict == Verdict::KeepUnstamped,
              "list at 14 days: 40 days unstamped is past twice the age, 10 days is not");
        Log("[prunetest] list at 14 days: beta.0000000b\\window_z14_1_2 (unstamped, 40 days) is %s, "
            "beta.0000000b\\cube16k (unstamped, 10 days) is %s",
            w ? VerdictName(w->verdict) : "missing", c ? VerdictName(c->verdict) : "missing");
    }

    // ---- 2. retire without prune.confirm: listed, nothing moved --------------------------------
    {
        Request r = q;
        r.mode = Mode::Retire;
        said.clear();
        const Result R0 = Run(r);
        const bool still = Same(before, SnapAll(root), true, &first);
        Check(R0.status == Status::Listed && still,
              "retire without prune.confirm lists and exits, nothing moved");
        Log("[prunetest] retire without prune.confirm: %s, %s", StatusName(R0.status),
            still ? "nothing moved" : "SOMETHING MOVED");
    }

    // ---- planted: a wrong confirm, another engine, an unreadable table, the ages --------------
    {
        Request r = q;
        r.mode = Mode::Retire;
        r.confirm = Narrow(run + L"\\trees2");
        said.clear();
        const Result R1 = Run(r);
        Caught(R1.status == Status::RefusedConfirm && Same(before, SnapAll(root), true, &first),
               "prune.confirm naming another folder: " + std::string(StatusName(R1.status)) +
                   ", nothing moved");
        r.confirm = Narrow(L.listing.root);
        r.processes = [](std::vector<Process>& rows) {
            rows.push_back({GetCurrentProcessId(), "gagame.exe"});
            rows.push_back({4242, "GAGAME.EXE"});
            return true;
        };
        const Result R2 = Run(r);
        Request p2 = r;
        p2.mode = Mode::Purge;
        const Result P2 = Run(p2);
        Caught(R2.status == Status::RefusedEngines && P2.status == Status::RefusedEngines &&
                   Same(before, SnapAll(root), true, &first),
               "another gagame.exe in the process table (GAGAME.EXE, pid 4242): retire and purge "
               "both refused, nothing moved or deleted");
        r.processes = [](std::vector<Process>&) { return false; };
        const Result R3 = Run(r);
        Caught(R3.status == Status::RefusedEngines && Same(before, SnapAll(root), true, &first),
               "a process table that cannot be read: refused, since no engine can be ruled out");
        r.processes = alone;
        int ages = 0;
        for (const double a : {0.5, std::nan(""), -3.0}) {
            Request ra = r;
            ra.ageDays = a;
            ages += Run(ra).status == Status::RefusedAge ? 1 : 0;
        }
        Request rp = r;
        rp.mode = Mode::Purge;
        rp.purgeDays = -1.0;
        ages += Run(rp).status == Status::RefusedAge ? 1 : 0;
        Caught(ages == 4 && Same(before, SnapAll(root), true, &first),
               "ages under their floors (0.5 days, NaN, -3 days; a purge age of -1) refused before "
               "the root is read");
    }

    // ---- 3. retire: the two stale folders move whole, with their .gaa; nothing else moves ------
    Result R;
    {
        Request r = q;
        r.mode = Mode::Retire;
        r.confirm = Narrow(L.listing.root);
        said.clear();
        R = Run(r);
        bool on = false;
        for (const std::string& s : said) {
            on |= s.rfind("retire:", 0) == 0;
            if (on) Log("[prunetest]   | %s", s.c_str());
        }
    }
    const std::wstring batch = retired + L"\\" + Widen(BatchName(now));
    const std::wstring movedA = alpha + L"\\window_z14_1_2", movedB = beta + L"\\window_z17_3_4";
    Check(R.status == Status::Done && R.batch == batch, "retire ends done, into .retired\\<now>");
    Check(R.moved.size() == 2 && !Exists(movedA) && !Exists(movedA + L".gaa") &&
              !Exists(movedA + L".gaa.stale") && !Exists(movedB) && !Exists(movedB + L".gaa.stale") &&
              Exists(batch + L"\\alpha.0000000a\\window_z14_1_2.gaa.stale") &&
              Exists(batch + L"\\beta.0000000b\\window_z17_3_4.gaa.stale"),
          "retire: exactly the STALE and the STALE-UNSTAMPED folder left, each with its archives");
    {
        // Every row as it was, the two moved subtrees now under the batch; folders' own write
        // times aside (a folder an entry left, and .retired, which gained the batch).
        const std::wstring relA = L"alpha.0000000a\\window_z14_1_2", relB = L"beta.0000000b\\window_z17_3_4";
        const std::wstring into = L".retired\\" + Widen(BatchName(now)) + L"\\";
        std::vector<Row> expect;
        for (Row r : before) {
            if (Starts(r.rel, relA) || Starts(r.rel, relB)) r.rel = into + r.rel;
            expect.push_back(r);
        }
        std::vector<Row> got;
        for (const Row& r : SnapAll(root)) {
            // the batch's own folders and its manifest are the only new entries
            const bool batchOwn = r.rel == into.substr(0, into.size() - 1) ||
                                  r.rel == into + L"alpha.0000000a" || r.rel == into + L"beta.0000000b" ||
                                  r.rel == into + L"manifest.json";
            if (!batchOwn) got.push_back(r);
        }
        std::sort(expect.begin(), expect.end(), [](const Row& a, const Row& b) { return a.rel < b.rel; });
        const bool exact = Same(expect, got, false, &first);
        Check(exact, "retire: every other entry where it was, byte for byte, and the moved ones "
                     "whole under the batch: " + first);
        Log("[prunetest] retire: 2 tag folders moved into .retired\\%s, each with its archives (the "
            "STALE one's .gaa and .gaa.stale, the STALE-UNSTAMPED one's .gaa.stale), one rename "
            "apiece; %zu entries compared, %s",
            BatchName(now).c_str(), expect.size(),
            exact ? "every other one byte for byte where it was" : "SOME DIFFER");
    }
    {
        std::string text;
        const HANDLE h = CreateFileW((batch + L"\\manifest.json").c_str(), GENERIC_READ,
                                     FILE_SHARE_READ, nullptr, OPEN_EXISTING, 0, nullptr);
        if (h != INVALID_HANDLE_VALUE) {
            char buf[16384];
            DWORD got = 0;
            if (ReadFile(h, buf, sizeof(buf), &got, nullptr)) text.assign(buf, got);
            CloseHandle(h);
        }
        std::string err;
        const JsonValue m = JsonParser::Parse(text, &err);
        const JsonValue* f = m.Get("folders");
        int named = 0;
        if (err.empty() && f && f->type == JsonValue::Type::Array) {
            for (const JsonValue& e : f->arr) {
                const JsonValue* undo = e.Get("undo");
                const size_t undos = undo && undo->type == JsonValue::Type::Array ? undo->arr.size() : 0;
                const bool fromA = e.Str("from") == Narrow(movedA) &&
                                   e.Str("gaaFrom") == Narrow(movedA + L".gaa") &&
                                   e.Str("staleFrom") == Narrow(movedA + L".gaa.stale") && undos == 3;
                const bool fromB = e.Str("from") == Narrow(movedB) && e.Get("gaaFrom") == nullptr &&
                                   e.Str("staleFrom") == Narrow(movedB + L".gaa.stale") && undos == 2;
                if ((fromA || fromB) && e.Str("state") == "moved" && e.Num("bytes", 0) > 0 &&
                    Exists(Widen(e.Str("to"))) && Exists(Widen(e.Str("staleTo"))) &&
                    !e.Str("verdict").empty()) {
                    ++named;
                }
            }
        }
        Check(named == 2 && m.Str("root") == Narrow(root),
              "the manifest names both folders and all their parts: from, to, the archives, bytes, "
              "verdict, state moved, one undo line a part");
    }
    const Skipped* jf = Skip(R.listing.skipped, gamma + L"\\cube16k");
    const bool junctionKept = jf && Has(jf->why, "SKIPPED whole") && Exists(gamma + L"\\cube16k\\link") &&
                              Exists(gamma + L"\\cube16k\\f0_m0_x0_y0.bin");
    Check(junctionKept, "retire: gamma.0000000c\\cube16k, 70 days stale, holds the junction: SKIPPED whole and says so");
    Check(Same(canary0, SnapAll(canary), true, &first), "retire: the canary untouched");
    Log("[prunetest] retire: gamma.0000000c\\cube16k (70 days stale, holding the junction) SKIPPED "
        "whole and said so: \"%s\"; the canary %s",
        jf ? jf->why.c_str() : "?", Same(canary0, SnapAll(canary), true, &first) ? "untouched" : "CHANGED");

    // ---- 4. purge: the 10-day batch goes; the 3-day, today's and the linked 12-day stay -------
    {
        Request p = q;
        p.mode = Mode::Purge;
        p.confirm = Narrow(L.listing.root);
        said.clear();
        const Result P = Run(p);
        bool on = false;
        for (const std::string& s : said) {
            on |= s.rfind("purge:", 0) == 0;
            if (on) Log("[prunetest]   | %s", s.c_str());
        }
        const Skipped* k = Skip(P.passed, b12);
        const Skipped* u = Skip(P.passed, b20);
        Check(P.status == Status::Done && !Exists(b10), "purge: the 10-day batch is gone");
        Check(Exists(b3) && Exists(batch) && Exists(retired + L"\\not-a-batch"),
              "purge: the 3-day batch, the batch just made and the folder that is not a batch remain");
        Check(k && Has(k->why, "reparse point") && Exists(b12 + L"\\gamma.0000000c\\cube16k\\f0_m0_x0_y0.bin") &&
                  Exists(b12 + L"\\gamma.0000000c\\cube16k\\link"),
              "purge: the 12-day batch holds a junction: SKIPPED whole, nothing in it deleted");
        Check(u && Has(u->why, "no manifest") && Exists(b20 + L"\\something-put-here-by-hand.bin"),
              "purge: the 20-day folder with no manifest is not retire's: SKIPPED whole");
        Check(Same(canary0, SnapAll(canary), true, &first), "purge: the canary untouched");
        Log("[prunetest] purge at 7 days: the 10-day batch deleted; the 3-day batch and the one just "
            "made remain; the 12-day batch holds a junction and the 20-day folder has no manifest, "
            "both SKIPPED whole; the canary %s",
            Same(canary0, SnapAll(canary), true, &first) ? "untouched" : "CHANGED");
    }

    // ---- 5. planted failures -------------------------------------------------------------------
    {
        // (a) The walker with the reparse check switched off, COUNTING ONLY.
        WalkStats on, off, del;
        on.record = off.record = true;
        const bool onOk = WalkTree(b12, Walk::Count, retired, on);
        WalkTree(b12, Walk::Count, retired, off, true);
        std::wstring reached;
        for (const std::wstring& v : off.visited) {
            if (Ends(v, L"\\link\\canary.bin")) reached = v;
        }
        bool onReached = false;
        for (const std::wstring& v : on.visited) onReached |= v.find(L"canary") != std::wstring::npos;
        Caught(onOk && !onReached && on.reparse.size() == 1 && !reached.empty(),
               "with the reparse check switched off the walker, counting only, reached " +
                   Narrow(reached.empty() ? std::wstring(L"nothing") : reached.substr(run.size() + 1)) +
                   Fmt(" (%llu files); with it on it stopped at the link (%llu files)",
                       static_cast<unsigned long long>(off.files),
                       static_cast<unsigned long long>(on.files)));
        const std::vector<Row> b12Rows = SnapAll(b12);
        const bool refused = !WalkTree(b12, Walk::Delete, retired, del, true);
        Caught(refused && Same(b12Rows, SnapAll(b12), true, &first) && Same(canary0, SnapAll(canary), true, &first),
               "the same switch with a delete is refused before anything is touched: \"" + del.why + "\"");
        WalkStats fence;
        const bool outside = !WalkTree(root, Walk::Delete, retired, fence);
        WalkStats linked;
        const bool whole = !WalkTree(b12, Walk::Delete, retired, linked) && Same(b12Rows, SnapAll(b12), true, &first);
        Caught(outside && whole && Exists(root) && Same(canary0, SnapAll(canary), true, &first),
               "a delete outside its fence, and one over a folder that holds a junction, both "
               "refused with nothing deleted");
    }
    {
        // (b) A root of out\...\google, whatever the confirm says.
        const std::wstring google = run + L"\\google";
        built = true;
        dir(google);
        dir(google + L"\\z.0000000f");
        dir(google + L"\\z.0000000f\\cube16k");
        file(google + L"\\z.0000000f\\cube16k\\f0_m0_x0_y0.bin", 70, tile);
        const std::vector<Row> g0 = SnapAll(google);
        Request g = q;
        g.mode = Mode::Retire;
        g.root = Narrow(google);
        g.confirm = Narrow(google);
        said.clear();
        const Result G = Run(g);
        Request d = q;
        d.root = "C:\\";
        const Result D = Run(d);
        Request r2 = q;
        r2.root = Narrow(run);
        const Result RR = Run(r2);
        Caught(built && G.status == Status::RefusedRoot && D.status == Status::RefusedRoot &&
                   RR.status == Status::RefusedRoot && Same(g0, SnapAll(google), true, &first),
               "a root of out\\...\\google is refused (\"" + G.why + "\"), and so are C:\\ and a "
               "folder not named trees");
    }
    {
        // (c) A root reached through a junction named cache, confirmed with the path as typed.
        const std::wstring elsewhere = run + L"\\elsewhere", link = run + L"\\cache";
        built = true;
        dir(elsewhere);
        dir(elsewhere + L"\\trees");
        dir(elsewhere + L"\\trees\\y.0000000e");
        dir(elsewhere + L"\\trees\\y.0000000e\\cube16k");
        file(elsewhere + L"\\trees\\y.0000000e\\cube16k\\f0_m0_x0_y0.bin", 70, tile);
        junction(link, elsewhere);
        const std::vector<Row> e0 = SnapAll(elsewhere);
        Request c = q;
        c.mode = Mode::Retire;
        c.root = Narrow(link + L"\\trees");
        c.confirm = c.root;
        said.clear();
        const Result C = Run(c);
        Caught(built && C.status == Status::RefusedConfirm && C.listing.root == elsewhere + L"\\trees" &&
                   Same(e0, SnapAll(elsewhere), true, &first),
               "a root through the junction named cache with prune.confirm set to the path as "
               "typed: refused -- the listing printed " +
                   Narrow(C.listing.root.substr(run.size() + 1)) + ", where a change would land");
    }
    {
        // (d) Failures stop the run at their item: a locked archive, a locked folder, a locked file.
        const std::wstring stop = run + L"\\stop\\trees";
        built = true;
        dir(run + L"\\stop");
        dir(stop);
        for (const wchar_t* n : {L"a.00000001", L"b.00000002", L"c.00000003"}) {
            dir(stop + L"\\" + n);
            dir(stop + L"\\" + n + L"\\cube16k");
            file(stop + L"\\" + n + L"\\cube16k\\f0_m0_x0_y0.bin", 70, tile);
        }
        file(stop + L"\\b.00000002\\cube16k.gaa", 70, tile);
        file(stop + L"\\b.00000002\\cube16k.gaa.stale", 70, tile);
        const std::wstring a = stop + L"\\a.00000001\\cube16k", b = stop + L"\\b.00000002\\cube16k",
                           c = stop + L"\\c.00000003\\cube16k";
        Request s = q;
        s.mode = Mode::Retire;
        s.root = Narrow(stop);
        s.confirm = Narrow(stop);
        said.clear();
        // b's archive set aside is held open with no delete-sharing: b's folder and its .gaa move,
        // its .gaa.stale cannot, and the two that moved must come back.
        HANDLE hold = CreateFileW((b + L".gaa.stale").c_str(), GENERIC_READ,
                                  FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr, OPEN_EXISTING, 0, nullptr);
        const Result S1 = Run(s);
        if (hold != INVALID_HANDLE_VALUE) CloseHandle(hold);
        const std::wstring batch1 = stop + L"\\.retired\\" + Widen(BatchName(now));
        Caught(built && hold != INVALID_HANDLE_VALUE && S1.status == Status::Stopped &&
                   S1.failedAt == b + L".gaa.stale" && Exists(batch1 + L"\\a.00000001\\cube16k") &&
                   !Exists(a) && Exists(b) && Exists(b + L".gaa") && Exists(b + L".gaa.stale") &&
                   Exists(c) && !Exists(batch1 + L"\\b.00000002\\cube16k") &&
                   !Exists(batch1 + L"\\b.00000002\\cube16k.gaa"),
               "an archive that will not move: the folder and the .gaa that had moved before it "
               "were put back and the run stopped there; the folder before it stayed moved, the "
               "one after it was not touched");
        // c's folder held open with no delete-sharing: its rename fails. A second later, a new batch.
        hold = CreateFileW(c.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr,
                           OPEN_EXISTING, FILE_FLAG_BACKUP_SEMANTICS, nullptr);
        s.now = now + kSecond;
        const Result S2 = Run(s);
        if (hold != INVALID_HANDLE_VALUE) CloseHandle(hold);
        const std::wstring batch2 = stop + L"\\.retired\\" + Widen(BatchName(now + kSecond));
        // (c's node folder in the batch was made before its rename was tried, and stays, empty.)
        Caught(hold != INVALID_HANDLE_VALUE && S2.status == Status::Stopped && S2.failedAt == c &&
                   Exists(batch2 + L"\\b.00000002\\cube16k") && Exists(batch2 + L"\\b.00000002\\cube16k.gaa") &&
                   Exists(batch2 + L"\\b.00000002\\cube16k.gaa.stale") && Exists(c) &&
                   Exists(c + L"\\f0_m0_x0_y0.bin") && !Exists(batch2 + L"\\c.00000003\\cube16k"),
               "a move that fails stops the run at that item: " + Narrow(S2.failedAt.substr(run.size() + 1)) +
                   " stayed, the one before it moved with both its archives");
        // A file held open inside the older batch: purge stops at it, the next batch untouched.
        const std::wstring lockedFile = batch1 + L"\\a.00000001\\cube16k\\f0_m0_x0_y0.bin";
        hold = CreateFileW(lockedFile.c_str(), GENERIC_READ, FILE_SHARE_READ | FILE_SHARE_WRITE, nullptr,
                           OPEN_EXISTING, 0, nullptr);
        Request p = s;
        p.mode = Mode::Purge;
        p.now = now + 10 * kDay;
        const std::vector<Row> second = SnapAll(batch2);
        const Result P1 = Run(p);
        if (hold != INVALID_HANDLE_VALUE) CloseHandle(hold);
        Caught(hold != INVALID_HANDLE_VALUE && P1.status == Status::Stopped && P1.failedAt == lockedFile &&
                   Exists(lockedFile) && Same(second, SnapAll(batch2), true, &first),
               "a delete that fails stops purge at that file; the next batch is untouched");
        const Result P3 = Run(p);
        Check(P3.status == Status::Done && !Exists(batch1) && !Exists(batch2),
              "the lock released, purge finishes both batches");
    }
    {
        // (e) The real process table can see an engine at all: this one.
        std::vector<Process> rows;
        const bool read = Processes(rows);
        bool self = false;
        for (const Process& p : rows) {
            std::string img = p.image;
            for (char& ch : img) ch = static_cast<char>(tolower(static_cast<unsigned char>(ch)));
            self |= p.pid == GetCurrentProcessId() && img.rfind("gagame", 0) == 0;
        }
        std::vector<Process> mine{{GetCurrentProcessId(), "gagame.exe"}};
        Check(read && self && !OtherEngine(mine, GetCurrentProcessId(), nullptr),
              "the real process table lists this engine, and the rule does not count it as another");
    }
    {
        // (f) The rules as tables: the root, the process image, the names.
        const std::wstring o = L"C:\\wt\\out";
        struct RootCase {
            const wchar_t* path;
            bool ok;
        };
        const RootCase roots[] = {{L"C:\\x\\cache\\trees", true},  {L"C:\\x\\CACHE\\Trees", true},
                                  {L"C:\\x\\cache\\trees\\", true}, {L"C:\\x\\cache\\google", false},
                                  {L"C:\\", false},                 {L"C:\\trees", false},
                                  {L"C:\\x\\trees", false},         {L"C:\\wt\\out\\a\\trees", true},
                                  {L"C:\\wt\\outside\\trees", false}, {L"C:\\wt\\out\\trees\\x", false}};
        int rootsRight = 0;
        for (const RootCase& rc : roots) rootsRight += RootAllowed(rc.path, o, nullptr) == rc.ok ? 1 : 0;
        const uint32_t self = 100;
        struct EngineCase {
            std::vector<Process> rows;
            bool other;
        };
        const EngineCase engines[] = {{{{100, "gagame.exe"}}, false},
                                      {{{100, "gagame.exe"}, {7, "GAGAME.EXE"}}, true},
                                      {{{9, "gagame_base.exe"}}, true},
                                      {{{5, "notepad.exe"}, {6, "gagame.pdb"}}, false},
                                      {{}, false}};
        int enginesRight = 0;
        for (const EngineCase& ec : engines) enginesRight += OtherEngine(ec.rows, self, nullptr) == ec.other ? 1 : 0;
        std::string nn, ni;
        struct NameCase {
            const wchar_t* name;
            int kind;   // 0 node, 1 tag, 2 tile
            bool ok;
        };
        const NameCase names[] = {{L"earth.color.5e9a5237", 0, true},
                                  {L"noaa.cudem.merrimack.0f5b0281", 0, true},
                                  {L"earth.color.composite", 0, false},
                                  {L"earth.color.5E9A5237", 0, false},
                                  {L".retired", 0, false},
                                  {L".abcdef12", 0, false},
                                  {L"x y.0000000a", 0, false},
                                  {L"cube16k", 1, true},
                                  {L"window_z14_1263360_1538048", 1, true},
                                  {L"cube16k.gaa", 1, false},
                                  {L"cube16k.gaa.stale", 1, false},
                                  {L".live", 1, false},
                                  {L"f0_m3_x12_y7.bin", 2, true},
                                  {L"f6_m0_x1_y2_0badc0de.bin", 2, true},
                                  {L"f7_m0_x3_y5_00000001.ref-0000000a", 2, true},
                                  {L"f0_m1_x0_y0.fold", 2, true},
                                  {L".live", 2, false},
                                  {L"~pub0000002a", 2, false},
                                  {L"f0_m3_x12.bin", 2, false}};
        int namesRight = 0;
        for (const NameCase& nc : names) {
            const bool got = nc.kind == 0 ? NodeName(nc.name, nn, ni) : nc.kind == 1 ? TagName(nc.name) : TileName(nc.name);
            namesRight += got == nc.ok ? 1 : 0;
        }
        uint64_t t = 0;
        const bool batchNames = ParseBatchName(BatchName(now), t) && t / kSecond == now / kSecond &&
                                !ParseBatchName("20260231T000000Z", t) && !ParseBatchName("not-a-batch", t);
        const size_t nr = sizeof(roots) / sizeof(roots[0]), ne = sizeof(engines) / sizeof(engines[0]),
                     nn2 = sizeof(names) / sizeof(names[0]);
        Check(rootsRight == int(nr) && enginesRight == int(ne) && namesRight == int(nn2) && batchNames,
              "the rules as tables");
        Log("[prunetest] the rules as tables: root %d of %zu, process image %d of %zu, names %d of %zu, "
            "batch names %s",
            rootsRight, nr, enginesRight, ne, namesRight, nn2, batchNames ? "round-trip" : "WRONG");
    }

    // ---- the scratch goes the way purge's batches go: unlink this test's junctions, then the walk
    if (g_ok) {
        bool unlinked = true;
        for (const std::wstring& l : links) {
            const DWORD a = GetFileAttributesW(l.c_str());
            if (a != INVALID_FILE_ATTRIBUTES && (a & FILE_ATTRIBUTE_REPARSE_POINT)) {
                unlinked &= RemoveDirectoryW(l.c_str()) != 0;   // the link, never its target
            }
        }
        const bool canaryLived = Same(canary0, SnapAll(canary), true, &first);
        WalkStats w;
        const bool gone = unlinked && canaryLived && WalkTree(run, Walk::Delete, base, w);
        Check(gone, "the scratch removed by the tool's own walk: " + w.why);
        if (gone) {
            Log("[prunetest] the scratch removed by the tool's own walk: %llu files in %llu folders, "
                "after its %zu junctions were unlinked and the canary was seen whole",
                static_cast<unsigned long long>(w.files), static_cast<unsigned long long>(w.folders),
                links.size());
        }
    }
    if (!g_ok) Log("[prunetest]   the scratch is left for inspection: %s", Narrow(run).c_str());
    Log("[prunetest] ---- %s: list changes nothing; retire moves exactly the stale tag folders with "
        "their archives and says how to undo it; purge deletes only old batches; every root, "
        "confirm, engine, link and failure planted above was refused ----",
        g_ok ? "PASS" : "FAIL");
    return g_ok;
}

}  // namespace ga
