// TreePrune - the tile trees' tag folders by last use; retire and purge only on confirm. The
// banner in TreePrune.h says what it must never do; this file is how it doesn't.
#include "compose/TreePrune.h"

#ifndef NOMINMAX
#define NOMINMAX   // windows.h's min/max macros would break std::min in every includer
#endif
#include <windows.h>
#include <tlhelp32.h>
#undef min
#undef max

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <map>

// tree_detail::WriteTile, the atomic publish, for the manifest. The other helpers there do not fit
// a tool that must know how every step came out: MakeDir and Delete drop the result, and Exists
// cannot tell a folder from a link to one.
#include "compose/TileTree.h"
#include "core/BuildInfo.h"
#include "core/Common.h"
#include "core/Json.h"

namespace ga::prune {

namespace {

using Say = std::function<void(const std::string&)>;

constexpr uint64_t kDay = 864000000000ull;   // FILETIME ticks (100 ns) in a day
constexpr int kMaxDepth = 16;                  // retire makes <batch>\<node>.<id>\<tag>: three

constexpr DWORD kLink = FILE_ATTRIBUTE_REPARSE_POINT;
constexpr DWORD kDir = FILE_ATTRIBUTE_DIRECTORY;

uint64_t Ticks(const FILETIME& f) {
    return (uint64_t(f.dwHighDateTime) << 32) | f.dwLowDateTime;
}
double Days(uint64_t now, uint64_t t) {
    return t >= now ? 0.0 : double(now - t) / double(kDay);   // a time ahead of the clock is new
}

std::string Fmt(const char* fmt, ...) {
    char b[1024];
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(b, sizeof(b), fmt, ap);
    va_end(ap);
    return b;
}
std::string Err(DWORD e) { return "err " + std::to_string(e); }
bool Refuse(std::string* why, const std::string& s) {
    if (why) *why = s;
    return false;
}

bool EqualCi(const std::wstring& a, const std::wstring& b) {
    return CompareStringOrdinal(a.c_str(), int(a.size()), b.c_str(), int(b.size()), TRUE) ==
           CSTR_EQUAL;
}
bool StartsCi(const std::wstring& s, const std::wstring& p) {
    return s.size() >= p.size() &&
           CompareStringOrdinal(s.c_str(), int(p.size()), p.c_str(), int(p.size()), TRUE) ==
               CSTR_EQUAL;
}
bool StrictlyInside(const std::wstring& p, const std::wstring& fence) {
    return fence.size() >= 3 && p.size() > fence.size() + 1 && StartsCi(p, fence + L"\\");
}
std::wstring TrimSlash(std::wstring p) {
    while (p.size() > 3 && (p.back() == L'\\' || p.back() == L'/')) p.pop_back();
    return p;
}
std::wstring Lower(std::wstring s) {
    for (wchar_t& c : s) c = static_cast<wchar_t>(towlower(c));
    return s;
}
// A path for the printout: below the root, relative to it; anywhere else, whole.
std::string Rel(const std::wstring& p, const std::wstring& root) {
    if (StrictlyInside(p, root)) return Narrow(p.substr(root.size() + 1));
    return Narrow(p);
}
// The ANSI spelling tree_detail's A functions take, or "" when the code page cannot carry it.
std::string Ansi(const std::wstring& w) {
    BOOL lossy = FALSE;
    const int n = WideCharToMultiByte(CP_ACP, WC_NO_BEST_FIT_CHARS, w.c_str(), int(w.size()),
                                      nullptr, 0, nullptr, &lossy);
    if (n <= 0 || lossy) return std::string();
    std::string s(size_t(n), '\0');
    WideCharToMultiByte(CP_ACP, WC_NO_BEST_FIT_CHARS, w.c_str(), int(w.size()), s.data(), n,
                        nullptr, &lossy);
    return lossy ? std::string() : s;
}
// A PowerShell literal: single quotes, a quote inside doubled.
std::string Ps(const std::wstring& p) {
    std::string o = "'";
    for (const char c : Narrow(p)) o += (c == '\'') ? std::string("''") : std::string(1, c);
    return o + "'";
}
// The line that undoes one move. Guarded, because Move-Item onto a folder that exists moves the
// source INTO it: a tree repainted in the meantime must stop the undo, not swallow the old one.
std::string Undo(const std::wstring& movedTo, const std::wstring& was) {
    return "if (-not (Test-Path -LiteralPath " + Ps(was) + ")) { Move-Item -LiteralPath " +
           Ps(movedTo) + " -Destination " + Ps(was) + " }";
}

struct Child {
    std::wstring name;
    DWORD attrs = 0;
    uint64_t size = 0, written = 0;
};
// A folder's entries, "." and ".." left out. FindFirstFileExW reports each entry's OWN
// attributes -- a junction shows FILE_ATTRIBUTE_REPARSE_POINT and nothing of what it points at --
// and its size and times come from the directory, so no entry is opened. They are the directory's
// COPY of them, which NTFS refreshes lazily, when a handle on the entry closes (the selftest met
// it: TreePruneTest.cpp Snap); a verdict here is counted in days. Called ON a link it would list
// the target, so every caller has checked the folder itself first.
bool Children(const std::wstring& dir, std::vector<Child>& out) {
    WIN32_FIND_DATAW fd{};
    const HANDLE h = FindFirstFileExW((dir + L"\\*").c_str(), FindExInfoBasic, &fd,
                                      FindExSearchNameMatch, nullptr, FIND_FIRST_EX_LARGE_FETCH);
    if (h == INVALID_HANDLE_VALUE) return false;
    do {
        if (wcscmp(fd.cFileName, L".") == 0 || wcscmp(fd.cFileName, L"..") == 0) continue;
        Child c;
        c.name = fd.cFileName;
        c.attrs = fd.dwFileAttributes;
        c.size = (uint64_t(fd.nFileSizeHigh) << 32) | fd.nFileSizeLow;
        c.written = Ticks(fd.ftLastWriteTime);
        out.push_back(std::move(c));
    } while (FindNextFileW(h, &fd));
    const DWORD err = GetLastError();
    FindClose(h);
    return err == ERROR_NO_MORE_FILES;
}

bool ParseUtc(const std::string& s, uint64_t& t) {
    int y = 0, mo = 0, d = 0, h = 0, mi = 0, se = 0;
    char z = 0;
    if (sscanf_s(s.c_str(), "%4d-%2d-%2dT%2d:%2d:%2d%c", &y, &mo, &d, &h, &mi, &se, &z, 1) != 7 ||
        z != 'Z') {
        return false;
    }
    SYSTEMTIME st{};
    st.wYear = WORD(y);
    st.wMonth = WORD(mo);
    st.wDay = WORD(d);
    st.wHour = WORD(h);
    st.wMinute = WORD(mi);
    st.wSecond = WORD(se);
    FILETIME f{};
    if (!SystemTimeToFileTime(&st, &f)) return false;
    t = Ticks(f);
    return true;
}

// The stamp's bytes: the one file the listing opens. Every share flag, so an engine publishing a
// new stamp over it is never refused (TileTree.h ReadTile gives the reason), and
// FILE_FLAG_OPEN_REPARSE_POINT, although a folder holding a link never gets this far.
void ReadStamp(Entry& e) {
    const std::wstring path = e.folder + L"\\.live";
    std::string text;
    const HANDLE h = CreateFileW(path.c_str(), GENERIC_READ,
                                 FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                                 OPEN_EXISTING, FILE_FLAG_OPEN_REPARSE_POINT, nullptr);
    if (h != INVALID_HANDLE_VALUE) {
        char buf[4096];
        DWORD got = 0;
        if (ReadFile(h, buf, sizeof(buf), &got, nullptr)) text.assign(buf, got);
        CloseHandle(h);
    }
    // The file's own time stands for the stamp whatever its text says, and the later of the two
    // is taken: a clock or a copy can only make a folder look NEWER, never staler.
    e.stampTime = e.stampWritten;
    std::string err;
    const JsonValue v = JsonParser::Parse(text, &err);
    uint64_t t = 0;
    if (err.empty() && ParseUtc(v.Str("utc"), t)) {
        e.stampRead = true;
        e.stampTime = (std::max)(t, e.stampWritten);
        e.stampRev = v.Str("rev");
        e.stampScene = v.Str("scene");
        e.stampFile = v.Str("file");
    }
}

bool IsStale(Verdict v) { return v == Verdict::Stale || v == Verdict::StaleUnstamped; }

Verdict Judge(const Entry& e, uint64_t now, double ageDays) {
    if (e.stamped) return Days(now, e.stampTime) <= ageDays ? Verdict::Keep : Verdict::Stale;
    return Days(now, e.newest) <= 2.0 * ageDays ? Verdict::KeepUnstamped : Verdict::StaleUnstamped;
}

// The two names beside a tag folder that are its own: the archive packed from it and the archive
// set aside from it. Anything else that starts "<tag>." is not understood, and the folder is then
// SKIPPED whole, so that no part of what might belong to it is ever separated from it.
const wchar_t* const kParts[] = {L".gaa", L".gaa.stale"};
bool Siblings(const std::vector<Child>& nodeKids, const std::wstring& tag, std::string* why) {
    const std::wstring stem = Lower(tag) + L".";
    for (const Child& c : nodeKids) {
        const std::wstring n = Lower(c.name);
        if (n.compare(0, stem.size(), stem) != 0) continue;
        if (n == Lower(tag) + kParts[0] || n == Lower(tag) + kParts[1]) continue;
        return Refuse(why, "beside it is " + Narrow(c.name) +
                               ", which is neither of its archives: SKIPPED whole, so nothing of it "
                               "is separated");
    }
    return true;
}

// One tag folder read and judged: its files, its stamp, its archives beside it. False = SKIPPED
// whole, with the reason. Used by the listing and again just before a move, so the folder that
// moves is the folder as it is then, not as it was listed. `nodeKids` is the node folder's
// listing, for the names beside it.
bool ReadTag(Entry& e, const std::vector<Child>& nodeKids, uint64_t folderWritten, uint64_t now,
             double ageDays, std::string* why) {
    if (!Siblings(nodeKids, Widen(e.tag), why)) return false;
    std::vector<Child> kids;
    if (!Children(e.folder, kids)) {
        return Refuse(why, "cannot be listed (" + Err(GetLastError()) + ")");
    }
    e.tiles = e.files = e.bytes = e.newest = 0;
    e.stamped = e.stampRead = false;
    e.gaa.clear();
    e.stale.clear();
    bool anyFile = false;
    for (const Child& c : kids) {
        if (c.attrs & kLink) {
            return Refuse(why, "holds a reparse point, " + Narrow(c.name) +
                                   ": SKIPPED whole, never entered");
        }
        if (c.attrs & kDir) {
            return Refuse(why, "holds a folder, " + Narrow(c.name) +
                                   ": a tag folder holds files, so it is SKIPPED whole");
        }
        ++e.files;
        e.bytes += c.size;
        if (EqualCi(c.name, L".live")) {
            e.stamped = true;
            e.stampWritten = c.written;
            continue;
        }
        anyFile = true;
        e.newest = (std::max)(e.newest, c.written);
        if (TileName(c.name)) ++e.tiles;
    }
    if (!anyFile) e.newest = folderWritten;   // an empty folder is as new as its making
    // Its archives sit beside it and move with it (review finding 25). Probed by name, as NTFS
    // resolves it, so the listing and the re-read before a move find the same files.
    for (int p = 0; p < 2; ++p) {
        const std::wstring path = e.folder + kParts[p];
        WIN32_FILE_ATTRIBUTE_DATA g{};
        if (!GetFileAttributesExW(path.c_str(), GetFileExInfoStandard, &g)) continue;
        if (g.dwFileAttributes & (kLink | kDir)) {
            return Refuse(why, "its archive " + e.tag + Narrow(kParts[p]) +
                                   " is not a plain file: SKIPPED whole");
        }
        (p == 0 ? e.gaa : e.stale) = path;
        e.bytes += (uint64_t(g.nFileSizeHigh) << 32) | g.nFileSizeLow;
        e.newest = (std::max)(e.newest, Ticks(g.ftLastWriteTime));
    }
    if (e.stamped) ReadStamp(e);
    e.verdict = Judge(e, now, ageDays);
    return true;
}

void ListNode(Listing& L, const std::wstring& nodeDir, const std::string& node,
              const std::string& id, uint64_t now, double ageDays) {
    std::vector<Child> kids;
    if (!Children(nodeDir, kids)) {
        L.skipped.push_back({nodeDir, "cannot be listed (" + Err(GetLastError()) + ")"});
        return;
    }
    std::map<std::wstring, bool> tags;   // lower-cased, so an archive finds its folder as NTFS does
    for (const Child& c : kids) {
        if ((c.attrs & kDir) && !(c.attrs & kLink) && TagName(c.name)) tags[Lower(c.name)] = true;
    }
    for (const Child& c : kids) {
        const std::wstring path = nodeDir + L"\\" + c.name;
        if (c.attrs & kLink) {
            L.skipped.push_back({path, "a reparse point (junction or symlink): not entered, not moved"});
            continue;
        }
        if (c.attrs & kDir) {
            if (!TagName(c.name)) {
                L.skipped.push_back({path, "not a tag folder's name"});
                continue;
            }
            Entry e;
            e.node = node;
            e.id = id;
            e.tag = Narrow(c.name);
            e.folder = path;
            std::string why;
            if (ReadTag(e, kids, c.written, now, ageDays, &why)) L.entries.push_back(std::move(e));
            else L.skipped.push_back({path, why});
            continue;
        }
        // A file: one of a tag folder's two archives belongs to that folder's entry.
        const std::wstring n = Lower(c.name);
        bool archive = false, owned = false;
        for (const wchar_t* part : kParts) {
            const size_t k = wcslen(part);
            if (n.size() > k && n.compare(n.size() - k, k, part) == 0) {
                archive = true;
                owned |= tags.count(n.substr(0, n.size() - k)) != 0;
            }
        }
        if (owned) continue;
        L.skipped.push_back({path, archive ? "an archive with no tag folder beside it"
                                           : "a file beside the tag folders"});
    }
}

void ListBatches(Listing& L, const std::wstring& retired, const Child& self) {
    if (self.attrs & kLink) {
        L.skipped.push_back({retired, "a reparse point: .retired is never followed, and retire "
                                      "and purge refuse it"});
        return;
    }
    if (!(self.attrs & kDir)) {
        L.skipped.push_back({retired, "not a folder: retire and purge refuse it"});
        return;
    }
    std::vector<Child> kids;
    if (!Children(retired, kids)) {
        L.skipped.push_back({retired, "cannot be listed (" + Err(GetLastError()) + ")"});
        return;
    }
    for (const Child& c : kids) {
        const std::wstring path = retired + L"\\" + c.name;
        uint64_t t = 0;
        if (c.attrs & kLink) {
            L.skipped.push_back({path, "a reparse point: never entered, never deleted"});
        } else if (!(c.attrs & kDir) || !ParseBatchName(Narrow(c.name), t)) {
            L.skipped.push_back({path, "not a batch (a folder named by the UTC stamp of a retire)"});
        } else {
            const DWORD m = GetFileAttributesW((path + L"\\manifest.json").c_str());
            const bool manifest = m != INVALID_FILE_ATTRIBUTES && !(m & (kLink | kDir));
            L.batches.push_back({Narrow(c.name), path, t, manifest});
        }
    }
}

std::string EntryLine(const Entry& e, uint64_t now) {
    std::string stamp = "unstamped";
    if (e.stamped) {
        stamp = Fmt("stamp %s (%.1f d)", Utc(e.stampTime).c_str(), Days(now, e.stampTime));
        if (e.stampRead) {
            stamp += " rev " + (e.stampRev.empty() ? std::string("?") : e.stampRev) + " scene " +
                     (e.stampScene.empty() ? std::string("?") : e.stampScene);
            if (!e.stampFile.empty()) stamp += " (" + e.stampFile + ")";
        } else {
            stamp += " [its text did not parse: the file's time stands]";
        }
    }
    // Which of its two archives stand beside it; their bytes are in the MB.
    const char* arch = !e.gaa.empty() && !e.stale.empty() ? "+gaa +stale"
                       : !e.gaa.empty()                   ? "+gaa"
                       : !e.stale.empty()                 ? "+stale"
                                                          : "";
    return Fmt("%-15s %-26s %s %-28s %7llu tiles %10.1f MB %-11s  newest %s (%.1f d)  ",
               VerdictName(e.verdict), e.node.c_str(), e.id.c_str(), e.tag.c_str(),
               static_cast<unsigned long long>(e.tiles), double(e.bytes) / 1048576.0, arch,
               Utc(e.newest).c_str(), Days(now, e.newest)) +
           stamp;
}

void PrintListing(const Listing& L, uint64_t now, double ageDays, double purgeDays, const Say& say) {
    uint64_t n[4] = {}, bytes[4] = {};
    for (const Entry& e : L.entries) {
        say(EntryLine(e, now));
        ++n[int(e.verdict)];
        bytes[int(e.verdict)] += e.bytes;
    }
    for (const Skipped& s : L.skipped) say("SKIPPED         " + Rel(s.path, L.root) + ": " + s.why);
    for (const Batch& b : L.batches) {
        const double d = Days(now, b.time);
        say(Fmt("retired batch %s: %.1f days old%s", b.name.c_str(), d,
                !b.manifest        ? " -- no manifest.json, so not a batch retire made: purge leaves it"
                : d > purgeDays ? " -- older than the purge age: purge would delete it"
                                : ""));
    }
    std::string t = Fmt("totals at %g days:", ageDays);
    for (int v = 0; v < 4; ++v) {
        t += Fmt("  %s %llu (%.1f MB)", VerdictName(Verdict(v)), static_cast<unsigned long long>(n[v]),
                 double(bytes[v]) / 1048576.0);
    }
    say(t + Fmt("  SKIPPED %zu", L.skipped.size()));
}

// Retire and purge act only when the key names the root exactly as the listing printed it.
bool Confirmed(const Request& q, Result& R, const Say& say) {
    const std::wstring& root = R.listing.root;
    if (q.confirm.empty()) {
        R.status = Status::Listed;
        say("prune.confirm is not set: nothing was changed. To go on, run again with --set "
            "prune.confirm=" + Narrow(root));
        return false;
    }
    if (!EqualCi(TrimSlash(Widen(q.confirm)), root)) {
        R.status = Status::RefusedConfirm;
        R.why = "prune.confirm is '" + q.confirm + "' and the root is '" + Narrow(root) + "'";
        say("REFUSED: " + R.why + " -- nothing was changed");
        return false;
    }
    return true;
}

// No other engine may be running: a tree in use must not move under its reader, and one that
// cannot be seen cannot be ruled out -- an unreadable process table refuses too.
bool EngineFree(const Request& q, std::string* why) {
    std::vector<Process> rows;
    const bool read = q.processes ? q.processes(rows) : Processes(rows);
    if (!read) return Refuse(why, "the process table could not be read, so no engine can be ruled out");
    std::string who;
    if (OtherEngine(rows, GetCurrentProcessId(), &who)) {
        return Refuse(why, "another engine is running, " + who +
                               ": a tree in use must not move under its reader");
    }
    return true;
}

void Refused(Result& R, Status s, const std::string& why, const Say& say,
             const char* changed = "nothing was changed") {
    R.status = s;
    R.why = why;
    say("REFUSED: " + why + " -- " + changed);
}
void Stop(Result& R, const std::wstring& at, const std::string& why, const Say& say) {
    R.status = Status::Stopped;
    R.failedAt = at;
    R.why = why;
    say("STOPPED at " + Rel(at, R.listing.root) + ": " + why +
        " -- every item after it is where it was");
}

// ---- retire -----------------------------------------------------------------------------------
// What moves for one tag folder: the folder, then each archive beside it, one rename apiece.
struct Part {
    std::wstring from, to;
};
std::vector<Part> PartsOf(const Entry& e, const std::wstring& to) {
    std::vector<Part> p{{e.folder, to}};
    if (!e.gaa.empty()) p.push_back({e.gaa, to + kParts[0]});
    if (!e.stale.empty()) p.push_back({e.stale, to + kParts[1]});
    return p;
}
std::string Belongings(const Entry& e) {
    std::string s;
    if (!e.gaa.empty()) s += " and " + e.tag + ".gaa";
    if (!e.stale.empty()) s += " and " + e.tag + ".gaa.stale";
    return s;
}
// Stuck: a part would not move AND a part that had moved would not go back -- the one state
// that leaves a tag folder's belongings apart, said with the lines that join them again.
enum class State : uint8_t { Planned, Moved, Passed, Failed, Stuck };
struct Item {
    Entry e;
    std::wstring to;       // <batch>\<node>.<id>\<tag>
    State state = State::Planned;
    std::string note;
};

// <batch>\manifest.json: every folder the run planned, from where, to where, its size and its
// verdict, and the line that undoes it -- written before the first move and again after the last,
// through the tree's own publish, so a reader never meets half of one.
bool WriteManifest(const Request& q, const Result& R, const std::string& name,
                   const std::vector<Item>& items, uint64_t now, std::string* why) {
    using tree_detail::JsonQuote;
    std::string j = "{\n  \"tool\": \"gagame tree-prune retire\",\n";
    j += "  \"rev\": " + JsonQuote(BuildGitRev()) + ",\n";
    j += "  \"root\": " + JsonQuote(Narrow(R.listing.root)) + ",\n";
    j += "  \"batch\": " + JsonQuote(name) + ",\n";
    j += "  \"utc\": " + JsonQuote(Utc(now)) + ",\n";
    j += "  \"ageDays\": " + Fmt("%g", q.ageDays) + ",\n";
    j += "  \"folders\": [\n";
    for (size_t i = 0; i < items.size(); ++i) {
        const Item& it = items[i];
        const Entry& e = it.e;
        const char* state = it.state == State::Moved    ? "moved"
                            : it.state == State::Passed ? "passed"
                            : it.state == State::Failed ? "failed"
                            : it.state == State::Stuck  ? "stuck"
                                                        : "planned";
        j += "    {\"node\": " + JsonQuote(e.node) + ", \"id\": " + JsonQuote(e.id) +
             ", \"tag\": " + JsonQuote(e.tag) + ", \"verdict\": " + JsonQuote(VerdictName(e.verdict)) +
             ", \"state\": " + JsonQuote(state) + ",\n";
        j += "     \"from\": " + JsonQuote(Narrow(e.folder)) + ", \"to\": " + JsonQuote(Narrow(it.to)) +
             ",\n";
        if (!e.gaa.empty()) {
            j += "     \"gaaFrom\": " + JsonQuote(Narrow(e.gaa)) + ", \"gaaTo\": " +
                 JsonQuote(Narrow(it.to + kParts[0])) + ",\n";
        }
        if (!e.stale.empty()) {
            j += "     \"staleFrom\": " + JsonQuote(Narrow(e.stale)) + ", \"staleTo\": " +
                 JsonQuote(Narrow(it.to + kParts[1])) + ",\n";
        }
        j += "     \"tiles\": " + std::to_string(e.tiles) + ", \"files\": " + std::to_string(e.files) +
             ", \"bytes\": " + std::to_string(e.bytes) + ", \"newest\": " + JsonQuote(Utc(e.newest)) +
             ", \"stamp\": " + (e.stamped ? JsonQuote(Utc(e.stampTime)) : std::string("null")) +
             ", \"note\": " + JsonQuote(it.note) + ",\n";
        j += "     \"undo\": [";
        const std::vector<Part> parts = PartsOf(e, it.to);
        for (size_t k = 0; k < parts.size(); ++k) {
            j += (k ? ", " : "") + JsonQuote(Undo(parts[k].to, parts[k].from));
        }
        j += "]}";
        j += (i + 1 < items.size()) ? ",\n" : "\n";
    }
    j += "  ]\n}\n";
    const std::string path = Ansi(R.batch + L"\\manifest.json");
    if (path.empty()) return Refuse(why, "the batch's path is not expressible in the ANSI code page");
    if (!tree_detail::WriteTile(path, std::vector<uint8_t>(j.begin(), j.end()), false)) {
        return Refuse(why, "cannot write " + path);
    }
    return true;
}

void RetireAll(const Request& q, Result& R, uint64_t now, const Say& say) {
    const std::wstring& root = R.listing.root;
    const std::string name = BatchName(now);
    const std::wstring retired = root + L"\\.retired", batch = retired + L"\\" + Widen(name);
    std::vector<Item> items;
    uint64_t bytes = 0;
    for (const Entry& e : R.listing.entries) {
        if (!IsStale(e.verdict)) continue;
        Item it;
        it.e = e;
        it.to = batch + L"\\" + Widen(e.node + "." + e.id) + L"\\" + Widen(e.tag);
        items.push_back(std::move(it));
        bytes += e.bytes;
    }
    say(Fmt("retire: %zu tag folders (%.1f MB) are STALE or STALE-UNSTAMPED; each moves with its "
            "archives, one rename apiece, into %s",
            items.size(), double(bytes) / 1048576.0, Rel(batch, root).c_str()));
    for (const Item& it : items) {
        say("  would move " + Rel(it.e.folder, root) + Belongings(it.e) + "  (" +
            VerdictName(it.e.verdict) + ")");
    }
    if (!Confirmed(q, R, say)) return;
    std::string why;
    if (!EngineFree(q, &why)) return Refused(R, Status::RefusedEngines, why, say);
    if (items.empty()) {
        R.status = Status::Done;
        say("nothing is stale: nothing moved");
        return;
    }
    // THE BATCH: .retired a plain folder on the root's volume (made when absent), the batch new.
    const DWORD ra = GetFileAttributesW(retired.c_str());
    if (ra != INVALID_FILE_ATTRIBUTES) {
        if ((ra & kLink) || !(ra & kDir)) {
            return Refused(R, Status::RefusedBatch, Narrow(retired) + " is not a plain folder", say);
        }
        if (!SameVolume(root, retired, &why)) return Refused(R, Status::RefusedVolume, why, say);
    } else if (!CreateDirectoryW(retired.c_str(), nullptr)) {
        return Refused(R, Status::RefusedBatch,
                       "cannot make " + Narrow(retired) + " (" + Err(GetLastError()) + ")", say);
    }
    if (!CreateDirectoryW(batch.c_str(), nullptr)) {
        return Refused(R, Status::RefusedBatch,
                       "cannot make the batch " + Narrow(batch) + " new (" + Err(GetLastError()) +
                           "): a second retire inside one second?",
                       say);
    }
    R.batch = batch;
    // From here a refusal leaves the empty batch folder behind, and says so; purge takes it.
    const char* leftEmpty = "nothing was moved; the empty batch folder is left for purge";
    if (!SameVolume(root, batch, &why)) return Refused(R, Status::RefusedVolume, why, say, leftEmpty);
    if (!WriteManifest(q, R, name, items, now, &why)) {
        return Refused(R, Status::RefusedBatch, why, say, leftEmpty);
    }
    for (Item& it : items) {
        if (!EngineFree(q, &why)) {
            it.state = State::Failed;
            it.note = why;
            Stop(R, it.e.folder, why, say);
            break;
        }
        // RE-READ AND RE-JUDGED just before the move: a run that started and exited since the
        // listing may have stamped it, and a folder that changed shape is not the one listed.
        WIN32_FILE_ATTRIBUTE_DATA fa{};
        if (!GetFileAttributesExW(it.e.folder.c_str(), GetFileExInfoStandard, &fa) ||
            (fa.dwFileAttributes & kLink) || !(fa.dwFileAttributes & kDir)) {
            it.state = State::Failed;
            it.note = "no longer a plain folder";
            Stop(R, it.e.folder, it.note, say);
            break;
        }
        Entry now2 = it.e;
        std::vector<Child> nodeKids;
        const std::wstring nodeDir = it.e.folder.substr(0, it.e.folder.find_last_of(L'\\'));
        if (!Children(nodeDir, nodeKids) ||
            !ReadTag(now2, nodeKids, Ticks(fa.ftLastWriteTime), now, q.ageDays, &why)) {
            it.state = State::Failed;
            it.note = "changed since the listing: " + why;
            Stop(R, it.e.folder, it.note, say);
            break;
        }
        if (!IsStale(now2.verdict)) {
            it.e = now2;
            it.state = State::Passed;
            it.note = std::string("used since the listing: ") + VerdictName(now2.verdict);
            R.passed.push_back({it.e.folder, it.note});
            say("PASSED " + Rel(it.e.folder, root) + ": " + it.note + " -- not moved");
            continue;
        }
        it.e = now2;
        const std::wstring nodeTo = batch + L"\\" + Widen(it.e.node + "." + it.e.id);
        if (!CreateDirectoryW(nodeTo.c_str(), nullptr) && GetLastError() != ERROR_ALREADY_EXISTS) {
            it.state = State::Failed;
            it.note = "cannot make " + Narrow(nodeTo) + " (" + Err(GetLastError()) + ")";
            Stop(R, it.e.folder, it.note, say);
            break;
        }
        // ONE RENAME PER PART, NEVER A COPY: without MOVEFILE_COPY_ALLOWED another volume fails
        // with ERROR_NOT_SAME_DEVICE instead of becoming a copy and a delete. The parts go
        // together or not at all: one that will not move sends the ones before it back, last
        // first, so a folder never leaves without the archives the tree reads before it.
        const std::vector<Part> parts = PartsOf(it.e, it.to);
        size_t k = 0;
        while (k < parts.size() && MoveFileExW(parts[k].from.c_str(), parts[k].to.c_str(), 0)) ++k;
        if (k < parts.size()) {
            const DWORD err = GetLastError();
            std::string stuck;
            for (size_t j = k; j-- > 0;) {
                if (!MoveFileExW(parts[j].to.c_str(), parts[j].from.c_str(), 0)) {
                    stuck += " " + Undo(parts[j].to, parts[j].from) + ";";
                }
            }
            it.state = stuck.empty() ? State::Failed : State::Stuck;
            it.note = Rel(parts[k].from, root) + " would not move (" + Err(err) + ")" +
                      (k == 0            ? std::string()
                       : stuck.empty() ? std::string("; what had moved before it was put back")
                                       : "; PARTS ARE LEFT IN THE BATCH -- move them back by hand:" + stuck);
            Stop(R, parts[k].from, it.note, say);
            break;
        }
        it.state = State::Moved;
        R.moved.push_back(it.e.folder);
        say("moved " + Rel(it.e.folder, root) + Belongings(it.e) + " -> " + Rel(it.to, root));
    }
    if (R.status != Status::Stopped) R.status = Status::Done;
    if (!WriteManifest(q, R, name, items, now, &why)) {
        say("the manifest could not be rewritten (" + why + "): the undo lines below are the record");
    }
    size_t n = 0;
    for (const Item& it : items) {
        if (it.state != State::Moved) continue;
        ++n;
        for (const Part& p : PartsOf(it.e, it.to)) say("  undo: " + Undo(p.to, p.from));
    }
    say(Fmt("retired %zu tag folders into %s; the manifest is %s", n, Rel(batch, root).c_str(),
            Rel(batch + L"\\manifest.json", root).c_str()));
}

// ---- purge ------------------------------------------------------------------------------------
void PurgeAll(const Request& q, Result& R, uint64_t now, const Say& say) {
    const std::wstring& root = R.listing.root;
    const std::wstring retired = root + L"\\.retired";
    std::vector<const Batch*> plan, unmade;
    for (const Batch& b : R.listing.batches) {
        if (Days(now, b.time) <= q.purgeDays) continue;
        (b.manifest ? plan : unmade).push_back(&b);
    }
    say(Fmt("purge: %zu retired batches are older than %g days; each is deleted whole or left whole",
            plan.size(), q.purgeDays));
    for (const Batch* b : plan) {
        say(Fmt("  would delete %s (%.1f days old)", Rel(b->folder, root).c_str(), Days(now, b->time)));
    }
    // A folder under .retired named like a batch but holding no manifest was not made by retire
    // (retire writes it before its first move), so whatever is in it was put there some other way.
    for (const Batch* b : unmade) {
        const std::string note = "no manifest.json, so not a batch retire made: SKIPPED whole";
        R.passed.push_back({b->folder, note});
        say("SKIPPED " + Rel(b->folder, root) + ": " + note);
    }
    if (!Confirmed(q, R, say)) return;
    std::string why;
    if (!EngineFree(q, &why)) return Refused(R, Status::RefusedEngines, why, say);
    if (plan.empty()) {
        R.status = Status::Done;
        say("no batch is older than the purge age: nothing deleted");
        return;
    }
    const DWORD ra = GetFileAttributesW(retired.c_str());
    if (ra == INVALID_FILE_ATTRIBUTES || (ra & kLink) || !(ra & kDir)) {
        return Refused(R, Status::RefusedBatch, Narrow(retired) + " is not a plain folder", say);
    }
    for (const Batch* b : plan) {
        if (!EngineFree(q, &why)) {
            Stop(R, b->folder, why, say);
            break;
        }
        // A COUNT FIRST. A batch that holds a link anywhere is left whole: nothing is deleted
        // around a junction that must itself stay, and nothing is ever deleted through one.
        WalkStats pre;
        if (!WalkTree(b->folder, Walk::Count, retired, pre)) {
            Stop(R, pre.failedAt.empty() ? b->folder : pre.failedAt, pre.why, say);
            break;
        }
        if (!pre.reparse.empty()) {
            const std::string note = "holds a reparse point, " + Rel(pre.reparse.front(), root) +
                                     ": SKIPPED whole, nothing in it deleted";
            R.passed.push_back({b->folder, note});
            say("SKIPPED " + Rel(b->folder, root) + ": " + note);
            continue;
        }
        WalkStats del;
        if (!WalkTree(b->folder, Walk::Delete, retired, del)) {
            Stop(R, del.failedAt, del.why + Fmt(" (%llu of its %llu files were deleted before it)",
                                                 static_cast<unsigned long long>(del.files),
                                                 static_cast<unsigned long long>(pre.files)),
                 say);
            break;
        }
        R.purged.push_back(b->folder);
        say(Fmt("deleted %s: %llu files in %llu folders, %.1f MB", Rel(b->folder, root).c_str(),
                static_cast<unsigned long long>(del.files), static_cast<unsigned long long>(del.folders),
                double(del.bytes) / 1048576.0));
    }
    if (R.status != Status::Stopped) R.status = Status::Done;
}

bool WalkAt(const std::wstring& folder, Walk mode, WalkStats& st, bool unsafeFollow, int depth) {
    auto fail = [&st](const std::wstring& at, const std::string& why) {
        st.failedAt = at;
        st.why = why;
        return false;
    };
    if (depth > kMaxDepth) return fail(folder, "deeper than anything retire makes: not walked");
    // AT EVERY LEVEL, BEFORE IT DESCENDS: the folder's OWN attributes, read now rather than
    // taken from the parent's listing. GetFileAttributesW does not follow a link.
    const DWORD a = GetFileAttributesW(folder.c_str());
    if (a == INVALID_FILE_ATTRIBUTES) return fail(folder, "cannot be read (" + Err(GetLastError()) + ")");
    if ((a & kLink) && !unsafeFollow) {
        st.reparse.push_back(folder);
        if (mode == Walk::Delete) return fail(folder, "a reparse point: not entered, not deleted");
        return true;
    }
    if (!(a & kDir)) return fail(folder, "not a folder");
    std::vector<Child> kids;
    if (!Children(folder, kids)) return fail(folder, "cannot be listed (" + Err(GetLastError()) + ")");
    ++st.folders;
    for (const Child& c : kids) {
        const std::wstring path = folder + L"\\" + c.name;
        if ((c.attrs & kLink) && !unsafeFollow) {
            st.reparse.push_back(path);
            if (mode == Walk::Delete) return fail(path, "a reparse point: not entered, not deleted");
            continue;
        }
        if (c.attrs & kDir) {
            if (!WalkAt(path, mode, st, unsafeFollow, depth + 1)) return false;
            continue;
        }
        if (mode == Walk::Count) {
            ++st.files;
            st.bytes += c.size;
            if (st.record) st.visited.push_back(path);
            continue;
        }
        if (!DeleteFileW(path.c_str())) return fail(path, "the delete failed (" + Err(GetLastError()) + ")");
        ++st.files;
        st.bytes += c.size;
    }
    if (mode == Walk::Delete && !RemoveDirectoryW(folder.c_str())) {
        return fail(folder, "the emptied folder would not go (" + Err(GetLastError()) + ")");
    }
    return true;
}

}  // namespace

// ---- names, times, text -----------------------------------------------------------------------
const char* VerdictName(Verdict v) {
    switch (v) {
        case Verdict::Keep: return "KEEP";
        case Verdict::Stale: return "STALE";
        case Verdict::KeepUnstamped: return "KEEP-UNSTAMPED";
        case Verdict::StaleUnstamped: return "STALE-UNSTAMPED";
    }
    return "?";
}
const char* StatusName(Status s) {
    switch (s) {
        case Status::Listed: return "listed";
        case Status::Done: return "done";
        case Status::RefusedAge: return "refused (age)";
        case Status::RefusedRoot: return "refused (root)";
        case Status::RefusedConfirm: return "refused (confirm)";
        case Status::RefusedEngines: return "refused (engines)";
        case Status::RefusedBatch: return "refused (batch)";
        case Status::RefusedVolume: return "refused (volume)";
        case Status::Stopped: return "stopped";
    }
    return "?";
}
int ExitCode(Status s) {
    if (s == Status::Listed || s == Status::Done) return 0;
    return s == Status::Stopped ? 1 : 2;
}

uint64_t Now() {
    FILETIME f{};
    GetSystemTimeAsFileTime(&f);
    return Ticks(f);
}
std::string Utc(uint64_t ft) {
    FILETIME f{};
    f.dwLowDateTime = static_cast<DWORD>(ft & 0xFFFFFFFFull);
    f.dwHighDateTime = static_cast<DWORD>(ft >> 32);
    SYSTEMTIME t{};
    if (ft == 0 || !FileTimeToSystemTime(&f, &t)) return "-";
    return Fmt("%04u-%02u-%02u %02u:%02uZ", t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute);
}
std::string BatchName(uint64_t ft) {
    FILETIME f{};
    f.dwLowDateTime = static_cast<DWORD>(ft & 0xFFFFFFFFull);
    f.dwHighDateTime = static_cast<DWORD>(ft >> 32);
    SYSTEMTIME t{};
    FileTimeToSystemTime(&f, &t);
    return Fmt("%04u%02u%02uT%02u%02u%02uZ", t.wYear, t.wMonth, t.wDay, t.wHour, t.wMinute, t.wSecond);
}
bool ParseBatchName(const std::string& n, uint64_t& ft) {
    if (n.size() != 16 || n[8] != 'T' || n[15] != 'Z') return false;
    for (size_t i = 0; i < 15; ++i) {
        if (i != 8 && (n[i] < '0' || n[i] > '9')) return false;
    }
    SYSTEMTIME st{};
    st.wYear = WORD(std::stoi(n.substr(0, 4)));
    st.wMonth = WORD(std::stoi(n.substr(4, 2)));
    st.wDay = WORD(std::stoi(n.substr(6, 2)));
    st.wHour = WORD(std::stoi(n.substr(9, 2)));
    st.wMinute = WORD(std::stoi(n.substr(11, 2)));
    st.wSecond = WORD(std::stoi(n.substr(13, 2)));
    FILETIME f{};
    if (!SystemTimeToFileTime(&st, &f)) return false;
    ft = Ticks(f);
    return BatchName(ft) == n;   // the round trip refuses a 31st of February
}

std::string Narrow(const std::wstring& w) {
    if (w.empty()) return std::string();
    const int n = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), int(w.size()), nullptr, 0, nullptr, nullptr);
    std::string s(size_t(n > 0 ? n : 0), '\0');
    if (n > 0) WideCharToMultiByte(CP_UTF8, 0, w.c_str(), int(w.size()), s.data(), n, nullptr, nullptr);
    return s;
}
std::wstring Widen(const std::string& s) {
    if (s.empty()) return std::wstring();
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(), int(s.size()), nullptr, 0);
    std::wstring w(size_t(n > 0 ? n : 0), L'\0');
    if (n > 0) MultiByteToWideChar(CP_UTF8, 0, s.c_str(), int(s.size()), w.data(), n);
    return w;
}

// ---- the root ---------------------------------------------------------------------------------
bool FinalPath(const std::wstring& path, std::wstring& out, std::string* why) {
    // No FILE_FLAG_OPEN_REPARSE_POINT here, on purpose: the ROOT may be reached through links
    // (the worktree's cache junction), and what is judged is where they land.
    const HANDLE h = CreateFileW(path.c_str(), FILE_READ_ATTRIBUTES,
                                 FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                                 OPEN_EXISTING, FILE_FLAG_BACKUP_SEMANTICS, nullptr);
    if (h == INVALID_HANDLE_VALUE) {
        return Refuse(why, Narrow(path) + " cannot be opened (" + Err(GetLastError()) + ")");
    }
    BY_HANDLE_FILE_INFORMATION bi{};
    const bool dir = GetFileInformationByHandle(h, &bi) && (bi.dwFileAttributes & kDir);
    wchar_t buf[1024];
    const DWORD n = GetFinalPathNameByHandleW(h, buf, 1024, FILE_NAME_NORMALIZED | VOLUME_NAME_DOS);
    CloseHandle(h);
    if (!dir) return Refuse(why, Narrow(path) + " is not a folder");
    if (n == 0 || n >= 1024) return Refuse(why, Narrow(path) + ": its final path cannot be read");
    out.assign(buf, n);
    if (out.compare(0, 8, L"\\\\?\\UNC\\") == 0) out = L"\\\\" + out.substr(8);
    else if (out.compare(0, 4, L"\\\\?\\") == 0) out = out.substr(4);
    out = TrimSlash(out);
    return true;
}

bool RootAllowed(const std::wstring& finalRoot, const std::wstring& outFinal, std::string* why) {
    const std::wstring r = TrimSlash(finalRoot);
    const size_t s1 = r.find_last_of(L'\\');
    if (s1 == std::wstring::npos || s1 == 0) {
        return Refuse(why, Narrow(r) + " is not a folder named trees under another folder");
    }
    if (!EqualCi(r.substr(s1 + 1), L"trees")) {
        return Refuse(why, Narrow(r) + " does not end in a folder named trees");
    }
    const std::wstring up = r.substr(0, s1);
    const size_t s2 = up.find_last_of(L'\\');
    if (EqualCi(s2 == std::wstring::npos ? up : up.substr(s2 + 1), L"cache")) return true;
    const std::wstring o = TrimSlash(outFinal);
    if (!o.empty() && StrictlyInside(r, o)) return true;
    return Refuse(why, Narrow(r) + " is neither ...\\cache\\trees nor a trees folder inside the "
                                   "worktree's out\\");
}

bool ResolveRoot(const std::string& given, std::wstring& finalRoot, std::string* why) {
    const std::wstring path = given.empty() ? std::wstring(L"cache\\trees") : Widen(given);
    if (!FinalPath(path, finalRoot, why)) return false;
    // The worktree's out\ counts only as the folder it is: an out\ that is itself a link could
    // make any folder named trees look like a scratch root.
    std::wstring outFinal;
    const DWORD oa = GetFileAttributesW(L"out");
    if (oa != INVALID_FILE_ATTRIBUTES && (oa & kDir) && !(oa & kLink)) {
        std::string ignored;
        if (!FinalPath(L"out", outFinal, &ignored)) outFinal.clear();
    }
    return RootAllowed(finalRoot, outFinal, why);
}

// ---- names ------------------------------------------------------------------------------------
bool NodeName(const std::wstring& n, std::string& node, std::string& id) {
    if (n.size() < 10 || n[n.size() - 9] != L'.') return false;
    const std::wstring hex = n.substr(n.size() - 8), name = n.substr(0, n.size() - 9);
    for (const wchar_t c : hex) {
        if (!((c >= L'0' && c <= L'9') || (c >= L'a' && c <= L'f'))) return false;
    }
    if (name.empty() || name[0] == L'.') return false;
    for (const wchar_t c : name) {   // tree_detail::Sanitize's alphabet
        const bool ok = (c >= L'0' && c <= L'9') || (c >= L'a' && c <= L'z') ||
                        (c >= L'A' && c <= L'Z') || c == L'.' || c == L'-' || c == L'_';
        if (!ok) return false;
    }
    node = Narrow(name);
    id = Narrow(hex);
    return true;
}
bool TagName(const std::wstring& n) {
    // Lattice::Tag: cube<k>k, or <kind>_z<zoom>_<x>_<y> -- a letter first, then letters, digits,
    // '_' and '-' (a negative origin). No dot, so no <tag>.gaa can pass for a folder.
    if (n.empty() || n.size() > 64) return false;
    if (!((n[0] >= L'a' && n[0] <= L'z') || (n[0] >= L'A' && n[0] <= L'Z'))) return false;
    for (const wchar_t c : n) {
        const bool ok = (c >= L'0' && c <= L'9') || (c >= L'a' && c <= L'z') ||
                        (c >= L'A' && c <= L'Z') || c == L'-' || c == L'_';
        if (!ok) return false;
    }
    return true;
}
bool TileName(const std::wstring& n) {
    // f<face>_m<mip>_x<x>_y<y>[_<key>].bin | .void | .fold | .ref-<id>, TileTree's Base().
    size_t i = 0;
    auto digits = [&] {
        const size_t s = i;
        while (i < n.size() && n[i] >= L'0' && n[i] <= L'9') ++i;
        return i > s;
    };
    auto lit = [&](const wchar_t* t) {
        const size_t k = wcslen(t);
        if (n.compare(i, k, t) != 0) return false;
        i += k;
        return true;
    };
    if (!lit(L"f") || !digits() || !lit(L"_m") || !digits() || !lit(L"_x") || !digits() ||
        !lit(L"_y") || !digits()) {
        return false;
    }
    if (i < n.size() && n[i] == L'_') {
        const size_t s = ++i;
        while (i < n.size() && ((n[i] >= L'0' && n[i] <= L'9') || (n[i] >= L'a' && n[i] <= L'f'))) ++i;
        if (i == s) return false;
    }
    if (i >= n.size() || n[i] != L'.') return false;
    const std::wstring ext = n.substr(i + 1);
    return ext == L"bin" || ext == L"void" || ext == L"fold" ||
           (ext.size() > 4 && ext.compare(0, 4, L"ref-") == 0);
}

// ---- engines ----------------------------------------------------------------------------------
bool OtherEngine(const std::vector<Process>& rows, uint32_t self, std::string* who) {
    for (const Process& p : rows) {
        std::string img = p.image;
        for (char& c : img) c = static_cast<char>(tolower(static_cast<unsigned char>(c)));
        // Any gagame*.exe, not only the one name: a copied binary (out\binA\gagame.exe keeps its
        // name; gagame_base.exe would not) reads the same trees.
        const bool engine = img.size() >= 10 && img.compare(0, 6, "gagame") == 0 &&
                            img.compare(img.size() - 4, 4, ".exe") == 0;
        if (engine && p.pid != self) {
            if (who) *who = p.image + " (pid " + std::to_string(p.pid) + ")";
            return true;
        }
    }
    return false;
}
bool Processes(std::vector<Process>& out) {
    const HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap == INVALID_HANDLE_VALUE) return false;
    PROCESSENTRY32W pe{};
    pe.dwSize = sizeof(pe);
    for (BOOL ok = Process32FirstW(snap, &pe); ok; ok = Process32NextW(snap, &pe)) {
        out.push_back({pe.th32ProcessID, Narrow(pe.szExeFile)});
    }
    CloseHandle(snap);
    return !out.empty();
}

bool SameVolume(const std::wstring& a, const std::wstring& b, std::string* why) {
    auto serial = [](const std::wstring& p, DWORD& s) {
        const HANDLE h = CreateFileW(p.c_str(), FILE_READ_ATTRIBUTES,
                                     FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE, nullptr,
                                     OPEN_EXISTING,
                                     FILE_FLAG_BACKUP_SEMANTICS | FILE_FLAG_OPEN_REPARSE_POINT, nullptr);
        if (h == INVALID_HANDLE_VALUE) return false;
        BY_HANDLE_FILE_INFORMATION bi{};
        const bool ok = GetFileInformationByHandle(h, &bi) != 0;
        CloseHandle(h);
        s = bi.dwVolumeSerialNumber;
        return ok;
    };
    DWORD sa = 0, sb = 0;
    if (!serial(a, sa) || !serial(b, sb)) {
        return Refuse(why, "the volumes of " + Narrow(a) + " and " + Narrow(b) + " cannot be read");
    }
    if (sa != sb) {
        return Refuse(why, Narrow(a) + " and " + Narrow(b) + " are on different volumes: a "
                           "rename cannot cross them and this tool never copies");
    }
    return true;
}

// ---- the listing, the walk, the run -----------------------------------------------------------
Listing List(const std::wstring& root, uint64_t now, double ageDays) {
    Listing L;
    L.root = root;
    std::vector<Child> kids;
    if (!Children(root, kids)) {
        L.skipped.push_back({root, "the root cannot be listed (" + Err(GetLastError()) + ")"});
        return L;
    }
    for (const Child& c : kids) {
        const std::wstring path = root + L"\\" + c.name;
        if (EqualCi(c.name, L".retired")) {
            ListBatches(L, path, c);   // never a tree
            continue;
        }
        if (c.attrs & kLink) {
            L.skipped.push_back({path, "a reparse point (junction or symlink): not entered, not moved"});
            continue;
        }
        if (!(c.attrs & kDir)) {
            L.skipped.push_back({path, "a file in the root, not a tree"});
            continue;
        }
        std::string node, id;
        if (!NodeName(c.name, node, id)) {
            L.skipped.push_back({path, "not named <name>.<8 lowercase hex>"});
            continue;
        }
        ListNode(L, path, node, id, now, ageDays);
    }
    std::sort(L.entries.begin(), L.entries.end(), [](const Entry& a, const Entry& b) {
        if (a.node != b.node) return a.node < b.node;
        if (a.id != b.id) return a.id < b.id;
        return a.tag < b.tag;
    });
    std::sort(L.skipped.begin(), L.skipped.end(),
              [](const Skipped& a, const Skipped& b) { return a.path < b.path; });
    std::sort(L.batches.begin(), L.batches.end(),
              [](const Batch& a, const Batch& b) { return a.time < b.time; });
    return L;
}

bool WalkTree(const std::wstring& folder, Walk mode, const std::wstring& fence, WalkStats& st,
              bool unsafeFollow) {
    if (mode == Walk::Delete && unsafeFollow) {
        st.failedAt = folder;
        st.why = "the reparse check cannot be switched off for a delete: nothing was touched";
        return false;
    }
    if (mode == Walk::Delete && !StrictlyInside(folder, fence)) {
        st.failedAt = folder;
        st.why = "not strictly inside " + Narrow(fence) + ": nothing was touched";
        return false;
    }
    if (mode == Walk::Delete) {
        // WHOLE OR NOT AT ALL, as far as links go: a delete that met a junction halfway would
        // stop there with its siblings already gone. So a count goes first, and any link in it
        // refuses the delete before one file is touched. (Purge counts too, to say which.)
        WalkStats pre;
        if (!WalkAt(folder, Walk::Count, pre, false, 0)) {
            st.failedAt = pre.failedAt;
            st.why = pre.why;
            return false;
        }
        if (!pre.reparse.empty()) {
            st.reparse = pre.reparse;
            st.failedAt = pre.reparse.front();
            st.why = "a reparse point under it: nothing was deleted";
            return false;
        }
    }
    return WalkAt(folder, mode, st, unsafeFollow, 0);
}

Result Run(const Request& q) {
    Result R;
    const Say say = q.say ? q.say : Say([](const std::string& s) { Log("[prune] %s", s.c_str()); });
    const uint64_t now = q.now ? q.now : Now();
    if (!std::isfinite(q.ageDays) || q.ageDays < 1.0) {
        Refused(R, Status::RefusedAge,
                Fmt("prune.ageDays is %g: it must be a number of days, at least 1 (under a day, "
                    "every tree a run has not touched since this morning would be stale)",
                    q.ageDays),
                say);
        return R;
    }
    if (!std::isfinite(q.purgeDays) || q.purgeDays < 0.0) {
        Refused(R, Status::RefusedAge,
                Fmt("prune.purgeDays is %g: it must be a number of days, at least 0", q.purgeDays), say);
        return R;
    }
    std::wstring root;
    std::string why;
    if (!ResolveRoot(q.root, root, &why)) {
        Refused(R, Status::RefusedRoot, why, say);
        return R;
    }
    const char* mode = q.mode == Mode::Retire ? "retire" : q.mode == Mode::Purge ? "purge" : "list";
    say("root " + (q.root.empty() ? std::string("cache\\trees") : q.root) + " -> " + Narrow(root) +
        "   (the full path prune.confirm must name to retire or purge)");
    say(Fmt("mode %s, now %s: a stamp older than %g days is STALE; with no stamp, nothing written in "
            "%g days is STALE-UNSTAMPED; retired batches older than %g days are purge's",
            mode, Utc(now).c_str(), q.ageDays, 2.0 * q.ageDays, q.purgeDays));
    R.listing = List(root, now, q.ageDays);
    PrintListing(R.listing, now, q.ageDays, q.purgeDays, say);
    if (q.mode == Mode::Retire) RetireAll(q, R, now, say);
    else if (q.mode == Mode::Purge) PurgeAll(q, R, now, say);
    else R.status = Status::Listed;
    return R;
}

}  // namespace ga::prune
